#include "motionvisualizer.h"
#include "constants.h"

using namespace ofxCv;

ofPolyline lineFacingNormal(const ofPolyline& input, const ofPoint& normal, float max_angle)
{
    ofPolyline output;
    float similarity = cos(max_angle);

    for (int ip = 0; ip < input.size(); ip++) {
        if (input.getNormalAtIndex(ip).dot(normal) > similarity) {
            output.addVertex(input[ip]);
        }
    }

    return output;
}

void MotionVisualizer::setup(FlowCam* in_flowcam)
{
    flowcam = in_flowcam;
    particles.setup(ofGetWidth(), ofGetHeight(), 4, 2048);
    particles.setTimeStep(1 / kFrameRate);
    trailhistory.clear();
    trailshapes.clear();
    trail_overlay = cv::Mat(flowcam->getFlowSize(), CV_8UC3);
    ofImage trail_texture_im;
    trail_texture_im.loadImage("assets/trail_texture.png");
    trail_texture = toCv(trail_texture_im).clone();
    // Fill the alpha timeline
    vector<ofPoint> alphas;
    alphas.push_back(ofPoint(0.0, 16));
    alphas.push_back(ofPoint(0.48, 16));
    alphas.push_back(ofPoint(0.5, 64));
    alphas.push_back(ofPoint(0.52, 4));
    alphas.push_back(ofPoint(2.0, 0.0));
    double t = 0;
    trail_alpha_timeline.clear();

    while (alphas.size() > 1) {
        double amt = ofMap(t, alphas[0].x, alphas[1].x, 0, 1);
        trail_alpha_timeline.push_back( ofLerp(alphas[0].y, alphas[1].y, amt) );
        t += trail_alpha_resolution;
        cout << t;

        if (t >= alphas[1].x) {
            alphas.erase(alphas.begin());
        }
    }

    trail_alpha_life = alphas[0].x;
}

void MotionVisualizer::update(double delta_t)
{
    t += delta_t;
    particles.update();
    particles.setupForces();
    particles.clean();
    updateFullTrails(delta_t);
//    updateTrailTexture(delta_t);
}


void MotionVisualizer::updateFullTrails(double delta_t)
{
    float scale_flow_to_game = ofGetWidth() / (float)flowcam->flow_high.cols;

    for (int ic = 0; ic < flowcam->contourfinder_low.size(); ic++) {
        const ofPolyline& contour = flowcam->contourfinder_low.getPolyline(ic).getResampledBySpacing(ofGetHeight() * 0.01);
        ofRectangle bbox = toOf(flowcam->contourfinder_low.getBoundingRect(ic));
        float inv_size = 1 / MIN(bbox.width, bbox.height) * scale_flow_to_game;
        ofVec2f center = toOf(flowcam->contourfinder_low.getCenter(ic));
        ofVec2f flow_dir = toOf(flowcam->flow.at<cv::Vec2f>(center.y, center.x));
        float magnitude = flow_dir.length();
        flow_dir.normalize();
        center *= scale_flow_to_game;
        const ofVec2f v = flow_dir.rotateRad(HALF_PI);
        ofPath path;

        for (int iv = 0; iv < contour.size(); iv ++) {
            path.lineTo(contour[iv] * scale_flow_to_game);
        }

        path.close();
        path.setFilled(true);
        ofMesh mesh = path.getTessellation();

        for (int im = 0; im < mesh.getNumVertices(); im ++) {
//            float d = ((mesh.getVertex(im) - center) * inv_size).dot(v);
            float d = ((mesh.getVertex(im) - center) * inv_size).y;
            float hue = ofWrap(d * 16 + 200, 0, 255);
            float saturation = 255;// * ofMap(magnitude, flowcam->flow_threshold_high, flowcam->flow_threshold_low, 0, 1);
            float brightness = 255;
            ofFloatColor color = ofColor::fromHsb(hue, saturation, brightness);
            mesh.addColor(color);
        }

        Trailshape shape = {0.0, mesh};
        trailshapes.push_back(shape);
    }

    for (int is = 0; is < trailshapes.size(); is ++) {
        trailshapes[is].t += delta_t;
    }

    while (trailshapes.size() && trailshapes[0].t > trail_alpha_life) {
        trailshapes.pop_front();
    }
}

void MotionVisualizer::updateTrailTexture(double delta_t)
{
    for (int ic = 0; ic < flowcam->contourfinder_low.size(); ic++) {
        const cv::Rect bbox = flowcam->contourfinder_low.getBoundingRect(ic);
        // Using overlays
        //        trail_overlay -= 1;
        const cv::Rect bbox_tex(0, 0, bbox.width, bbox.height);
        cv::Mat section = trail_overlay(bbox);
        cv::Mat mask_section = flowcam->flow_low(bbox);
        cv::Mat tex_section = trail_texture(bbox_tex);
        tex_section.copyTo(section, mask_section);
    }

    trail_overlay -= cv::Scalar(3, 1, 2);
}

void MotionVisualizer::draw()
{
    ofSetColor(200, 200, 200, 255);
    flowcam->frame_screen_im.draw(0, 0, ofGetWidth(), ofGetHeight());
//    drawTrailTexture();
    drawTrailShapes();
    particles.draw();
}

void MotionVisualizer::drawTrailTexture()
{
    // Color multiply
    ofSetColor(255, 255, 255, 255);
    vector<cv::Mat> channels;
    //    channels.push_back(flowcam->flow_hist);
    //    channels.push_back(flowcam->flow_hist);
    //    channels.push_back(cv::Mat(flowcam->flow_low.rows, flowcam->flow_low.cols, CV_8UC1, cv::Scalar(255)));
    //    cv::Mat colormap;
    //    cv::merge(channels, colormap);
    //    blur(colormap, colormap, 20);
    ofEnableBlendMode(OF_BLENDMODE_ADD);
    ofxCv::drawMat(trail_overlay, 0, 0, ofGetWidth(), ofGetHeight());
    ofDisableBlendMode();
}

void MotionVisualizer::drawTrailShapes()
{
    ofPushMatrix();
    ofEnableBlendMode(OF_BLENDMODE_ADD);

    for (int is = 0; is < trailshapes.size(); is ++) {
        Trailshape& shape = trailshapes[is];
        const double n = shape.t / trail_alpha_resolution;
        const int left = n;
        const int right = n + 1;
        const double amt = n - left;
        const int alpha = ofLerp(trail_alpha_timeline[left], trail_alpha_timeline[right], amt);

        for (int im = 0; im < shape.shape.getNumVertices(); im ++) {
            shape.shape.setColor(im, ofFloatColor(shape.shape.getColor(im), alpha / 255.0));
        }

        ofSetColor(ofColor::white, alpha);
//        shape.shape.setUseShapeColor(false);
//        shape.shape.setFilled(true);
        shape.shape.draw();
    }

    ofDisableBlendMode();
    ofPopMatrix();
}

void MotionVisualizer::trail(ofPoint pos, ofPoint dir, float radius)
{
    float angle = atan2(dir.y, dir.x);
    ofPoint spread = dir;
    spread = spread.rotate(0, 0, 90.0);
    pos = (pos + spread * 0.5 * ofRandom(-radius, radius)) + (radius * dir);
    ofPoint vel = dir * 50 + spread * ofRandom(-30, 30);
    Particle p(pos.x, pos.y, angle);
    p.xv = vel.x;
    p.yv = vel.y;
    p.shape = SHAPE_RECT;
    p.life = ofRandom(3.0);
    p.size = radius / 10 * ofRandom(1.0, 3.0);
    p.color = ofColor::fromHsb(ofRandom(256.0), 0.0, 255);
    particles.add(p);
}

void MotionVisualizer::sparkle(ofPoint pos, float radius)
{
    for (int i = 0; i < 40; i ++) {
        Particle p(pos.x, pos.y);
        p.a = ofRandom(TWO_PI);
        ofPoint vel = ofPoint(cos(p.a), sin(p.a)) * radius * ofRandom(5, 10);
        p.xv = vel.x;
        p.yv = vel.y;
        p.life = ofRandom(4, 8);
        p.damping = 0.9;
        p.size = radius / 10;
        p.shape = SHAPE_RECT;
        p.color = ofColor::fromHex(0xE224B6);
        particles.add(p);
    }
}