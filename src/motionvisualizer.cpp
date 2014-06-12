#include "motionvisualizer.h"
#include "constants.h"

using namespace ofxCv;

void MotionVisualizer::setup(FlowCam* in_flowcam)
{
    flowcam = in_flowcam;
    particles.setup(ofGetWidth(), ofGetHeight(), 4, 2048);
    particles.setTimeStep(1 / kFrameRate);
}

void MotionVisualizer::update(double delta_t)
{
//    particles.add(Particle(ofGetWidth() / 2 + ofRandomf(), ofGetHeight() / 2 + ofRandomf()));
    particles.update();
    particles.setupForces();
    particles.clean();
    
    updateTrails();
//    updateGlitch();
}

void MotionVisualizer::updateTrails()
{
    float scale_flow_to_frame = ofGetWidth() / (float)flowcam->flow_high.cols;
    const vector<unsigned int>& current = flowcam->contourfinder_low.getTracker().getCurrentLabels();

    for (vector<unsigned int>::const_iterator it = current.begin(); it != current.end(); ++it) {
        unsigned int label = *it;
        ofPolyline& found = trails[label];
        const cv::Rect& bb = flowcam->contourfinder_low.getTracker().getSmoothed(label);
        ofPoint p(bb.x + bb.width / 2, bb.y + bb.height / 2);
        found.lineTo(p * scale_flow_to_frame);
        cout << p << endl;
    }
}

void MotionVisualizer::updateGlitch()
{
    float scale_flow_to_frame = (float)flowcam->frame.cols / (float)flowcam->flow_high.cols;

    for (int ci = 0; ci < flowcam->contourfinder_high.size(); ci++) {
        // Check distance from last freeze-frame
        unsigned int label = flowcam->contourfinder_high.getTracker().getLabelFromIndex(ci);
        ofPoint& found = freezes[label];
        cv::Rect bounds = flowcam->contourfinder_high.getBoundingRect(ci);
        ofPoint center = ofPoint(bounds.x + bounds.width / 2, bounds.y + bounds.width / 2);

        if (found.distance(center) > 50) {
            ofPolyline contour = flowcam->contourfinder_high.getPolyline(ci).getResampledBySpacing(20);

            for (int pi = 0; pi < contour.size(); pi++) {
                ofPoint p = contour[pi];
                ofPoint n = contour.getNormalAtIndex(pi) * 20;
                particles.add(Particle(p.x * scale_flow_to_frame, p.y * scale_flow_to_frame, 0.0, n.x, n.y));
            }
        }
    };

//    const vector<unsigned int>& current = flowcam->contourfinder_high.getTracker().getCurrentLabels();
//    for (vector<unsigned int>::const_iterator it = current.begin(); it != current.end(); ++it){
//        unsigned int label = *it;
//        ofPoint& lastpos = freezes[label];
//        flowcam->contourfinder_high.s
//    }
}



void MotionVisualizer::draw()
{
    particles.draw();
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

void MotionVisualizer::sparkle(ofPoint pos, float radius){
    for (int i = 0; i < 40; i ++){
        Particle p(pos.x, pos.y);
        p.a = ofRandom(TWO_PI);
        ofPoint vel = ofPoint(cos(p.a), sin(p.a)) * radius * ofRandom(5,10);
        p.xv = vel.x;
        p.yv = vel.y;
        p.life = ofRandom(4,8);
        p.damping = 0.9;
        p.size = radius / 10;
        p.shape = SHAPE_RECT;
        p.color = ofColor::fromHex(0xE224B6);
        particles.add(p);
    }
}