#include "motionvisualizer.h"
#include "constants.h"

using namespace ofxCv;

void MotionVisualizer::setup(FlowCam* in_flowcam){
    flowcam = in_flowcam;
    particles.setup(ofGetWidth(), ofGetHeight(), 4, 2048);
    particles.setTimeStep(1 / kFrameRate);
}

void MotionVisualizer::update(double delta_t){
//    particles.add(Particle(ofGetWidth() / 2 + ofRandomf(), ofGetHeight() / 2 + ofRandomf()));

    particles.update();
    particles.setupForces();
    particles.clean();
    
//    updateTrails();
    updateGlitch();
    
}

void MotionVisualizer::updateTrails(){
    if (ofRandomuf() < 0.1){
        trails.erase(trails.begin());
    }
    float scale_flow_to_frame = ofGetWidth() / (float)flowcam->flow_high.cols;
    const vector<unsigned int>& current = flowcam->contourfinder_low.getTracker().getCurrentLabels();
    for (vector<unsigned int>::const_iterator it = current.begin(); it != current.end(); ++it){
        unsigned int label = *it;
        ofPolyline& found = trails[label];
        const cv::Rect& bb = flowcam->contourfinder_low.getTracker().getSmoothed(label);
        ofPoint p(bb.x + bb.width / 2, bb.y + bb.height/2);
        found.lineTo(p * scale_flow_to_frame);
        cout << p << endl;
    }
}

void MotionVisualizer::updateGlitch(){
    float scale_flow_to_frame = (float)flowcam->frame.cols / (float)flowcam->flow_high.cols;
    
    for (int ci = 0; ci < flowcam->contourfinder_high.size(); ci++){
        // Check distance from last freeze-frame
        unsigned int label = flowcam->contourfinder_high.getTracker().getLabelFromIndex(ci);
        ofPoint& found = freezes[label];
        cv::Rect bounds = flowcam->contourfinder_high.getBoundingRect(ci);
        ofPoint center = ofPoint(bounds.x + bounds.width / 2, bounds.y + bounds.width / 2);
        
        if (found.distance(center) > 50){
            ofPolyline contour = flowcam->contourfinder_high.getPolyline(ci).getResampledBySpacing(20);
            for (int pi = 0; pi < contour.size(); pi++){
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



void MotionVisualizer::draw(){
    
    ofEnableAlphaBlending();

    ofSetCircleResolution(6);
    ofSetLineWidth(2.0f);
    ofNoFill();
    for (int i = 0; i < particles.size(); i ++) {
        Particle & p = particles[i];
        ofSetColor(0, 220, 220, 80);
        ofCircle(p.x, p.y, 10);
    }
    
    for (map<unsigned int, ofPolyline>::const_iterator it = trails.begin(); it != trails.end(); ++it){
        ofPolyline p = it->second;
//        p.setColor(ofColor::white);
//        p.setFilled(true);
//        p.setStrokeWidth(2.0f);
        p.draw();
        cout << p.size() << endl;
    }
}

void MotionVisualizer::squidFlee(ofPoint pos, ofPoint dir){
//    fluid->addTemporalForce(pos, dir * scale * 25, main_color, 0.1 * body_radius * scale,  1.0f);
}