
#ifndef SLOW_SQUID_H_
#define SLOW_SQUID_H_

////////// IMPORTS //////////
#include "utilities.h"

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxPathfinder.h"

#include "Box2D/Box2D.h"

#include <vector>

using std::vector;

////////// CONSTANTS //////////



////////// CLASS DEF //////////

class Squid
{
public:
    // CONST
    const cv::Size kPathGridSize = cv::Size(20, 10);
    const cv::Size kSectionsSize = cv::Size(4, 2);
    
    // Settings
    float local_flow_high = 0.3f;
    double push_force = 100000.0;
    double tentacle_prep_force = 400.0f;
    double tentacle_damping = 10.0f;
    double motion_time_prep = 0.2;
    double motion_time_push = 0.4;
    double min_velocity = 200;
    double max_goal_distance = 80;
    double max_face_distance = 10;
    int num_tentacles = 9;
    int num_segments = 4;
    double segment_length = 30;
    double body_radius = 40;
    double body_density = 0.2;
    
    float face_search_window = 0.2;
    float face_size_min = 0.6;  // This is relative to the full frame, not the face search window
    float face_size_max = 0.05; // This is relative to the full frame, not the face search window
    
    enum BehaviorState { IDLE, PANIC, FACE };
    enum MotionState { STILL, PREP, PUSH, GLIDE };
    
    // MEMBERS
    b2Body* body = NULL;
    vector <b2Body *> tentacles;
    vector <b2RevoluteJoint *> tentacle_joints;
    ofxPathfinder pathfinder;
    cv::Point goal_section;
    cv::Mat grid, sections;
    ofImage grid_im;
    ofxCv::ObjectFinder objectfinder;

    BehaviorState behavior_state = IDLE;
    MotionState motion_state = STILL;
    
    ofPoint pos_game;
    ofPoint pos_grid;
    double motion_time = 0.0f;
    ofPoint goal;
    ofPoint waypoint_direction;
    double waypoint_distance;
    cv::Rect local_area;
    cv::Rect face_roi;
    cv::Mat face_mat;
    ofRectangle found_face;
    bool has_face = false;
    double frame_scale = 1.0;

    // METHODS
    void setup(ofPtr<b2World> phys_world);
    void setupPhysics(ofPtr<b2World> phys_world);
    
    void update(double delta_t, cv::Mat flow_high, cv::Mat frame);
    void updateObjectFinder(cv::Mat frame);
    
    void selectQuietGoal();
    void selectFaceGoal();
    
    void grabFace(cv::Mat frame);
    
    void findWaypoint();
    void bodyPush(double delta_t);
    void bodyPrep(double delta_t);
    void tentaclePrep();
    void tentaclePush();
    void tentacleGlide();
    void idleMotion(double delta_t);
    void draw(bool draw_debug);
    
    ofPoint getPosition() { return b2ToOf(body->GetPosition()); };
};

#endif