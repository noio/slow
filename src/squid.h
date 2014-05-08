
#ifndef SLOW_SQUID_H_
#define SLOW_SQUID_H_

////////// IMPORTS //////////

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxPathfinder.h"

#include "Box2D/Box2D.h"

#include <vector>

using std::vector;

////////// CONSTANTS //////////

const cv::Size kPathGridSize = cv::Size(20, 10);
const cv::Size kSectionsSize = cv::Size(4, 2);
const double kPushForce = 100000.0;
const double kPushTime = 0.4;
const double kPrepTime = 0.2;
const double kMinVelocity = 200;
const double kMaxGoalDistance = 100;
const int kNumTentacles = 7;
const int kNumSegments = 4;
const double kTentacleSegmentLength = 20;
const double kBodyRadius = 20;

////////// CLASS DEF //////////

class Squid
{
public:
    // MEMBERS
    b2Body* body;
    vector <b2Body *> tentacles;
    vector <b2RevoluteJoint *> tentacle_joints;
    ofxPathfinder pathfinder;
    int goal_x = 0, goal_y = 0;
    cv::Point goal_section;
    cv::Mat grid, sections;
    ofImage grid_im;
    
    // State
    double pushing = 0.0f;
    ofPoint push_direction;

    // METHODS
    void setup(ofPtr<b2World> phys_world);
    void update(double delta_t, cv::Mat flow_high, bool draw_debug);
    void draw();
};

#endif