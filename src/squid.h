
#ifndef SLOW_SQUID_H_
#define SLOW_SQUID_H_

#include "ofMain.h"
#include "ofxBox2d.h"
#include "ofxCv.h"

#include <vector>

using std::vector;

class Squid
{

public:
    ofxBox2dCircle body;

    ofPoint path_grid_size = ofPoint(16, 8);
    int path_region = 5;

    vector<ofPoint> target_path;

    void setup(ofxBox2d& box2d);
    void update(cv::Mat flow_high, bool draw_debug);
    void draw();

};

#endif