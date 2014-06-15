
#ifndef SLOW_MOTIONVISUALIZER_H_
#define SLOW_MOTIONVISUALIZER_H_

#include "ParticleSystem.h"
#include "flowcam.h"
#include "ofxCv.h"
#include <iostream>
#include <map>
#include <vector>
#include <deque>

using std::map;
using std::vector;
using std::deque;

typedef struct Trailshape {
    double t;
    ofPath shape;
} Trailshape;

typedef struct Trailtail {
    unsigned int length;
    ofPolyline tail;
    ofPoint direction;
} Trailtail;

ofPolyline lineFacingNormal(const ofPolyline& input, const ofPoint& normal, float max_angle);

class MotionVisualizer {
public:
    void setup(FlowCam* in_flowcam);
    void update(double delta_t);
    void draw();
    void trail(ofPoint pos, ofPoint dir, float radius);
    void sparkle(ofPoint pos, float radius);
    
    double max_trail_life = 1.6;
    double trail_fade_in = 0.5;
    int trail_max_alpha = 255;

private:
    void updateTrails(double delta_t);
    void updateGlitch();
    void glitchRegion(cv::Rect region);
    
    ParticleSystem particles;
    FlowCam* flowcam;
    
    map<unsigned int, Trailtail> trailhistory;
    deque<Trailshape> trailshapes;
    cv::Mat trail_overlay;
    cv::Mat trail_texture;

};

#endif /* defined(SLOW_MOTIONVISUALIZER_H_) */
