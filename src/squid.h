
#ifndef SLOW_SQUID_H_
#define SLOW_SQUID_H_

////////// IMPORTS //////////
#include "utilities.h"

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxPathfinder.h"
#include "ofxFluid.h"
#include "ofxPlaylist.h"

#include "Box2D/Box2D.h"

#include <vector>

using std::vector;

////////// CONSTANTS //////////


////////// CLASS DEF //////////

class Squid
{
public:
    // CONST
    const cv::Size kSectionsSize = cv::Size(8, 4);
    
    enum BehaviorState { IDLE, PANIC, FACE };
    enum MotionState { STILL, PREP, PUSH, GLIDE, LOCK };

    // Settings
    
    float scale = 1.0f;
    int num_tentacles = 7;
    int num_segments = 7;
    ofPoint tentacle_attach_scale = ofPoint(0.80, 0.2);
    ofPoint tentacle_attach_offset = ofPoint(0.0, 0.175);
    float segment_join_length = 0.9;
    float segment_length = 20.0;
    float segment_width = 6.0;
    float tentacle_density = 0.01f;
    double tentacle_damping = 3.0f;
    
    double body_radius = 40;
    double body_density = 0.2;
    double body_damping = 4.0f;
    
    double push_force = 40.0;
    double panic_force_multiplier = 2.0;
    double tentacle_prep_force = 400.0f;

    double motion_time_prep = 0.15;
    double motion_time_push = 0.3;
    double idle_move_cooldown = 10.0;
    double face_pose_time = 5.0;
    
    double min_velocity = 150;
    float max_goal_distance_close = 60;
    float max_goal_distance_far = 200;
    double max_face_distance = 10;
    float goal_bottom_margin = 2.0; // relative to body_radius
    
    int face_detection_threshold = 3;
    
    float face_grab_padding = 2.0;
    
    float local_flow_min = 0.05f;
    float local_flow_max = 0.1f;
    
    double face_search_window = 0.2;
    double face_size_min = 0.6;  // This is relative to the full frame, not the face search window
    double face_size_max = 0.05; // This is relative to the full frame, not the face search window
    
    // MEMBERS
    b2Body* body = NULL;
    vector <b2Body *> tentacles;
    vector <b2RevoluteJoint *> tentacle_joints;
    ofImage body_front_im, body_back_im, body_accent_im, tentacle_front_im, tentacle_back_im, hint_im, face_mask_im;
    ofxCv::ObjectFinder objectfinder;
    ofxPlaylist playlist;
    cv::Mat frame;
    cv::Mat face_mask_mat;
    cv::Rect face_roi;
    cv::Mat face_mat;
    ofImage face_im;
    ofxFluid* fluid;

    BehaviorState behavior_state = IDLE;
    MotionState motion_state = STILL;
    double time_in_motion_state = 0.0;
    double time_in_behavior_state = 0.0;
    
    ofPoint pos_game, pos_section;
    
    ofRectangle found_face;
    bool has_face = false;
    int face_detection_count = 0;
    double frame_scale = 1.0;

    ofPoint goal;
    cv::Point2i goal_section;
    cv::Rect local_area;
    
    cv::Mat sections;
    float local_flow;
    
    float squish = 1.0f;
    
    float hint_alpha = 0.0f;
    float hint_progress = 0.0f;
    
    ofColor main_color;
    
    // Helpers
    ofPoint goal_direction;
    float goal_angle, goal_distance;
    float going_slow;
    bool on_goal;
    bool facing_goal;
    bool sees_face;

    // METHODS
    void setup(ofPtr<b2World> phys_world, ofxFluid* fluid);
    void setupPhysics(ofPtr<b2World> phys_world);
    
    void update(double delta_t, cv::Mat flow_high, cv::Mat frame, ofxFluid& fluid);
    void updateObjectFinder(cv::Mat frame);
    
    void updateBehaviorState(double delta_t);
    void updateMotionState(double delta_t);
    
    void switchBehaviorState(BehaviorState next);
    void switchMotionState(MotionState next);
    
    void selectQuietGoalInRegion(cv::Rect bounds);
    void selectQuietGoal();
    void selectFaceGoal();
    bool currentGoalIsQuiet();
    
    void grabFace(bool do_cut);
    
    void bodyPush(double delta_t);
    void turnToAngle(float target_angle, double delta_t);
    void turnToGoal(double delta_t);
    void turnUpright(double delta_t);
    void squishPrep(double delta_t);
    void squishPush(double delta_t);
    void squirt();
    void tentaclePrep();
    void tentacleGlide();
    
    void draw(bool draw_debug);
    void drawBody();
    void drawTentacles();
    void drawDebug();
    
    ofPoint getPosition() { return b2ToOf(body->GetPosition()); };
};

#endif