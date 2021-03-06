
#ifndef SLOW_MOTIONVISUALIZER_H_
#define SLOW_MOTIONVISUALIZER_H_

#include "ParticleSystem.h"
#include "flowcam.h"
#include "ofxCv.h"
#include "ofxDropstuff.h"
#include <iostream>
#include <map>
#include <vector>
#include <deque>

using std::map;
using std::vector;
using std::deque;

typedef struct Trailshape {
    double t;
    ofMesh shape;
} Trailshape;


ofPolyline lineFacingNormal(const ofPolyline& input, const ofPoint& normal, float max_angle);

class MotionVisualizer {
public:
    void setup(ofxDS::FlowCam* in_flowcam);
    void update(double delta_t);
    void draw();
    void trail(ofPoint pos, ofPoint dir, float radius);
    void sparkle(ofPoint pos, float radius);
    
    int trail_hue = 200;
    int trail_hue_range = 64;
    float trail_alpha_mtp = 1.0;
    
private:
    void updateFullTrails(double delta_t);
    
    void drawTrailShapes();
    
    ParticleSystem particles;
    ofxDS::FlowCam* flowcam;
    
    double t;
    vector<float> trail_alpha_timeline;
    double trail_alpha_resolution = 0.1;
    double trail_alpha_life;
    
    deque<Trailshape> trailshapes;

};

#endif /* defined(SLOW_MOTIONVISUALIZER_H_) */
