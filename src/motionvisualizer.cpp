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
}

void MotionVisualizer::update(double delta_t)
{
//    particles.add(Particle(ofGetWidth() / 2 + ofRandomf(), ofGetHeight() / 2 + ofRandomf()));
    particles.update();
    particles.setupForces();
    particles.clean();
    updateTrails(delta_t);
//    updateGlitch();
}

void MotionVisualizer::updateTrails(double delta_t)
{
    float scale_flow_to_game = ofGetWidth() / (float)flowcam->flow_high.cols;

    for (int ic = 0; ic < flowcam->contourfinder_low.size(); ic++) {
        unsigned int label = flowcam->contourfinder_low.getLabel(ic);
        ofPolyline& contour = flowcam->contourfinder_low.getPolyline(ic);

        // If this is the first contour on this trail we need to estimate the
        // "backside" using the flow.
        if (trailhistory.find(label) == trailhistory.end()) {
            cv::Vec2f center = flowcam->contourfinder_low.getCenter(ic);
            ofPoint flow_dir = toOf(flowcam->flow.at<cv::Vec2f>(center[1], center[0])).normalized();
            const ofPolyline& backside = lineFacingNormal(contour, flow_dir, PI / 4);

            if (backside.size()) {
                Trailtail tail = {0, backside.getResampledByCount(10)};
                trailhistory[label] = tail;
            }
        } else {
            Trailtail& prev = trailhistory.at(label);
            ofPoint flow_dir = (contour.getCentroid2D() - prev.tail.getCentroid2D()).normalized();
            const ofPolyline& backside = lineFacingNormal(contour, flow_dir, PI / 4).getResampledByCount(10);
            ofLogVerbose("MotionVisualizer") << "add to existing trail" << backside.size();

            for (int step = 0; step < backside.size() - 1 && step < prev.tail.size() - 1; step++) {
                ofPath trail;
                const double progress = step / 10.0;
                const double progress_n = (step + 1) / 10.0;
                ofLogVerbose("MotionVisualizer") << backside.size();
                trail.moveTo(prev.tail[step] * scale_flow_to_game);
                trail.lineTo(backside[step] * scale_flow_to_game);
                trail.lineTo(backside[step + 1] * scale_flow_to_game);
                trail.lineTo(prev.tail[step + 1] * scale_flow_to_game);
                trail.close();
                trail.setFilled(true);
                trail.setFillColor(ofColor::fromHsb(240 - prev.length - step * 4, 255, 255));
                const Trailshape shape = {0.0, trail};
                trailshapes.push_back(shape);
            }

            prev.tail = backside;
            prev.length ++;
        }
    }

    for (int is = 0; is < trailshapes.size(); is ++) {
        trailshapes[is].t += delta_t;
    }

    while (trailshapes.size() && trailshapes[0].t > max_trail_life) {
        trailshapes.pop_front();
    }

    const vector<unsigned int>& dead_labels = flowcam->contourfinder_low.getTracker().getDeadLabels();

    for (int i = 0; i < dead_labels.size(); i++) {
        trailhistory.erase(dead_labels[i]);
    }
}


void MotionVisualizer::draw()
{
    particles.draw();
    ofPushMatrix();
//    float scale_flow_to_game = ofGetWidth() / (float)flowcam->flow_high.cols;
//    ofScale(scale_flow_to_game, scale_flow_to_game);
//    for (map<unsigned int, ofPolyline>::iterator it = trailhistory.begin(); it != trailhistory.end(); ++it){
//        it->second.draw();
//    }
    ofNoFill();

    for (int is = 0; is < trailshapes.size(); is ++) {
        Trailshape& shape = trailshapes[is];
        ofSetColor(shape.shape.getFillColor(), 255 * (1 - (shape.t / max_trail_life)));
        shape.shape.setUseShapeColor(false);
        shape.shape.setFilled(false);
        shape.shape.getOutline()[0].draw();
    }

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