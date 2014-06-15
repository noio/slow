
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
    
private:
    void updateBackTrails(double delta_t);
    void updateFullTrails(double delta_t);
    void updateTrailTexture(double delta_t);
    
    void drawTrailTexture();
    void drawTrailShapes();
    
    ParticleSystem particles;
    FlowCam* flowcam;
    
    double t;
    vector<float> trail_alpha_timeline;
    double trail_alpha_resolution = 0.1;
    double trail_alpha_life;
    
    map<unsigned int, Trailtail> trailhistory;
    deque<Trailshape> trailshapes;
    cv::Mat trail_overlay;
    cv::Mat trail_texture;

};

#endif /* defined(SLOW_MOTIONVISUALIZER_H_) */
