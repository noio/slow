
#ifndef SLOW_SQUID_H_
#define SLOW_SQUID_H_

////////// IMPORTS //////////
#include "utilities.h"
#include "flowcam.h"
#include "motionvisualizer.h"
#include "framerecord.h"
#include "highscoretable.h"
#include "objectfinderthreaded.h"

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxFluid.h"
#include "ofxPlaylist.h"

#include "Box2D/Box2D.h"

#include <vector>

using std::vector;
using std::deque;

typedef struct SquidColorPreset {
    ofColor body;
    ofColor body_outline;
    ofColor tentacles;
    ofColor tentacles_outline;
    ofColor markings;
} SquidColorPreset;

////////// CONSTANTS //////////

const ofColor kSquidPink = ofColor::fromHex(0xCF44D3);
const ofColor kSquidOutlineWhite = ofColor::fromHex(0xFFFFFF);
const ofColor kSquidDefaultColor = ofColor::fromHex(0x4A86FF);
const ofColor kSquidGreen = ofColor::fromHex(0x30d672);
const ofColor kSquidTransparentWhite(255,255,255,96);
const SquidColorPreset kDefaultColors = {kSquidDefaultColor, kSquidOutlineWhite, kSquidDefaultColor, kSquidOutlineWhite, kSquidDefaultColor};
const SquidColorPreset kPanicColors = {kSquidPink, kSquidOutlineWhite, kSquidPink, kSquidOutlineWhite, kSquidPink};
const SquidColorPreset kGrabColors = {kSquidGreen, kSquidGreen, kSquidTransparentWhite, kSquidTransparentWhite, kSquidTransparentWhite};

////////// CLASS DEF //////////


class Squid
{
public:
    // CONST
    const cv::Size kSectionsSize = cv::Size(8, 4);
    
    enum BehaviorState { IDLE, PANIC, FACE, GRABBED, STATIONARY, BORED };
    enum MotionState { STILL, PREP1, PREP2, PUSH, GLIDE, LOCK };
    
    Squid(){};
    Squid(const Squid&) = delete;            // no copy
    Squid& operator=(const Squid&) = delete; // no assign

    
    // METHODS
    void setup(ofPtr<b2World> phys_world, FlowCam* flowcam, MotionVisualizer* visualizer, HighscoreTable* in_highscores);
    void update(double delta_t);
    void draw(bool draw_debug);
    
    void stayAtPoint(const ofPoint& target, double duration);
    void switchColors(const SquidColorPreset& switch_to, float duration);
    void switchColorsTemp(const SquidColorPreset& switch_to, float duration, float period);
    void switchColorsTemp(const ofColor& body_fill, const ofColor& body_outline, const ofColor& tentacle_fill, const ofColor& tentacle_outline, const ofColor& markings, float duration, float period);

    
    // GETTERS & SETTERS
    ofPoint getPosition() const;
    float getBodyAngle() const;
    ofPoint getGoalDirection() const;
    double getLastActivity() const;
    void setScale(float in_scale);
    float getTimeLastFace() const {return time_last_face;};
    std::string getState();

    // PUBLIC SETTINGS
    
    double push_force = 40.0;
    double panic_force_multiplier = 2.0;
    double tentacle_prep_force = 160.0f;
    double tentacle_push_force = 80.0f;
    double turn_torque = 60.0;

    double motion_time_prep1 = 0.15;
    double motion_time_prep2 = 0.1;
    double motion_time_push = 0.3;
    double idle_move_cooldown = 10.0;
    double face_pose_time = 5.0;
    double grab_time = 4.0;
    double stationary_time = 5.0;
    double inactivity_time_until_bored = 120.0;
    double bored_time = 6.0; // Time spent in BORED state
    
    double min_velocity = 150;
    float max_goal_distance = 30;
    double max_face_distance = 10;
    float goal_padding = 1.0; // relative to body_radius * scale
    
    int face_detection_threshold = 3;
    
    float face_grab_padding = 2.0;
    
    float local_area_radius = 100;
    float core_area_radius = 40;
    
    float local_flow_min = 0.01f;
    float local_flow_max = 0.02f;
    
    double face_search_window = 0.2;
    double face_size_min = 0.6;  // This is relative to the full frame, not the face search window
    double face_size_max = 0.05; // This is relative to the full frame, not the face search window
    
    const ofColor kIdleColor = ofColor::fromHex(0x53A8BF);
    const ofColor kFaceColor = ofColor::fromHex(0xDAEBEF);
    const ofColor kPanicColor = ofColor::fromHex(0xE5411A);
    const ofColor kGrabColor = ofColor::fromHex(0x76CC32);

    const ofColor kBodyColor = ofColor(137,202,217);
    const ofColor kOutlineColor = ofColor(255,255,255);

private:
    void setupPhysics();
    void setupTextures();
    
    void updateFlow();
    void updateFinder();
    void updateBehaviorState(double delta_t);
    void updateMotionState(double delta_t);
    
    void switchBehaviorState(BehaviorState next);
    void switchMotionState(MotionState next);
    
    void selectQuietGoalInRegion(cv::Rect bounds);
    void selectQuietGoalAdjacent();
    void selectQuietGoal();
    void moveGoalWithFlow();
    void selectFaceGoal();
    bool currentGoalIsQuiet();
    void setGoal(const ofPoint& in_goal);
    void setGoal(float x, float y);
    
    void clearFace();
    void grabFace();
    
    void showCaptureHint();
    
    void bodyPush(double delta_t);
    void turnToAngle(float target_angle, double delta_t);
    void turnToGoal(double delta_t);
    void turnUpright(double delta_t);
    void squishPrep(double delta_t);
    void squishPush(double delta_t);
    void squirt();
    void tentaclePrep();
    void tentaclePush();
    void tentacleGlide();
    
    void tweenColors();
    void drawScore();
    void drawBody();
    void drawTentacles();
    void drawDebug();
    
    // PRIVATE MEMBERS

    float scale = 1.0f;
    vector<ofPoint> tentacle_attach;
    vector<ofPoint> tentacle_out_direction;
    int num_segments = 2;
    float segment_join_length = 0.95;
    float segment_length = 32.0;
    float segment_width = 20.0;
    float tentacle_density = 0.1f;
    double tentacle_damping = 3.0f;
    
    double body_radius = 40;
    double body_density = 0.2;
    double body_damping = 4.0f;
    
    // MEMBERS
    ofPtr<b2World> phys_world;
    b2Body* body = NULL;
    vector <b2Body *> tentacles;
    vector <b2RevoluteJoint *> tentacle_joints;
    ofImage body_base_back_im, body_base_front_im, body_base_front_outline_im, body_bubble_im, body_bubble_outline_im, markings_bubble_im;
    ofImage hint_im, face_mask_im;
    ofTrueTypeFont score_font;

    ObjectFinderThreaded objectfinder;
    ofxPlaylist playlist;
    cv::Mat face_mask_mat;
    cv::Rect face_roi;
    ofPtr<FrameRecord> face_anim;
    FlowCam* flowcam;
    MotionVisualizer *visualizer;
    HighscoreTable* highscores;
    
    BehaviorState behavior_state = IDLE;
    MotionState motion_state = STILL;
    double time_in_motion_state = 0.0;
    double time_in_behavior_state = 0.0;
    float time_last_active = 0.0;
    float time_last_face = 0.0;
    
    ofPoint pos_game, pos_section;
    
    ofRectangle found_face;
    bool has_face = false;
    double face_time = 0.0;
    int face_detection_count = 0;
    bool waiting_for_face_results = false;
    bool search_for_face = false;
    double scale_frame_to_game = 1.0;
    
    cv::Mat sections;
    ofPoint goal;
    cv::Point2i goal_section;

    ofRectangle local_area;
    ofRectangle core_area;
    
    float local_flow = 0.0f;
    float core_flow = 0.0f;
    
    float squish = 1.0f;
    
    float hint_alpha = 0.0f;
    float hint_progress = 0.0f;
    
    SquidColorPreset colors_cur;
    SquidColorPreset colors_prev;
    SquidColorPreset colors_tweened;

    float color_prev_amount = 0.0;
    
    // Helpers
    ofPoint goal_direction;
    float goal_angle, goal_distance;
    float going_slow;
    bool on_goal;
    bool facing_goal;
    bool sees_face;

};

#endif