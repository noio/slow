
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
    void trail(ofPoint pos, ofPoint dir, float radius);
    void sparkle(ofPoint pos, float radius);

private:
    void updateTrails();
    void updateGlitch();
    void glitchRegion(cv::Rect region);
    
    ParticleSystem particles;
    FlowCam* flowcam;

};

#endif /* defined(__slow__motionvisualizer__) */
