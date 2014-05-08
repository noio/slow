
#ifndef SLOW_SQUID_H_
#define SLOW_SQUID_H_

////////// IMPORTS //////////

#include "ofMain.h"
#include "ofxBox2d.h"
#include "ofxCv.h"
#include "ofxPathfinder.h"

#include <vector>

using std::vector;

////////// CONSTANTS //////////

const cv::Size kPathGridSize = cv::Size(20, 10);
const cv::Size kSectionsSize = cv::Size(4, 2);

////////// CLASS DEF //////////

class Squid
{
public:
    // MEMBERS
    ofxBox2dCircle body;
    ofxPathfinder pathfinder;
    int goal_x = 0, goal_y = 0;
    cv::Point goal_section;
    cv::Mat grid, sections;
    ofImage grid_im;

    // METHODS
    void setup(ofxBox2d& box2d);
    void update(cv::Mat flow_high, bool draw_debug);
    void draw();
};

#endif