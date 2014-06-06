
#ifndef SLOW_MOTIONVISUALIZER_H_
#define SLOW_MOTIONVISUALIZER_H_

#include "ParticleSystem.h"
#include "flowcam.h"
#include "ofxCv.h"
#include <iostream>
#include <map>

using std::map;

class MotionVisualizer {
public:
    void setup(FlowCam* in_flowcam);
    void update(double delta_t);
    void draw();
    void squidFlee(ofPoint pos, ofPoint dir);

private:
    void updateTrails();
    void updateGlitch();
    void glitchRegion(cv::Rect region);
    
    ParticleSystem particles;
    FlowCam* flowcam;
    map<unsigned int, ofPolyline> trails;
    map<unsigned int, ofPoint> freezes;
    

};

#endif /* defined(__slow__motionvisualizer__) */
