#pragma once

#include "Squid.h"
#include "Fluid.h"
#include "ParticleSystem.h"

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxUI.h"
#include "ofxFluid.h"
#include "ofxDelaunay.h"
#include "Box2D/Box2D.h"


class ofApp : public ofBaseApp
{

public:

    void setup();
    void update();
    void draw();
    void exit();
    
    void reset();
    
    void setupPhysics();
    void setupGUI();
    
    void doCapture();

    void updateFrame();
    void updateFlow();
    
    void drawMotionEffects();
    void drawFluid();
    void drawJaggies();
    
    ofColor getPersistentColor(int i);

    void keyPressed(int key);
    void keyReleased(int key);
    void mouseMoved(int x, int y );
    void mouseDragged(int x, int y, int button);
    void mousePressed(int x, int y, int button);
    void mouseReleased(int x, int y, int button);
    void windowResized(int w, int h);
    void dragEvent(ofDragInfo dragInfo);
    void gotMessage(ofMessage msg);
    void guiEvent(ofxUIEventArgs& e);

    // Components
    ofVideoPlayer video;
    ofVideoGrabber camera;
    
    ofImage motion_texture_a;

    ofxCv::FlowFarneback opticalflow;
    ofxCv::ContourFinder contourfinder;
    ofxFluid fluid;
    
    ofPtr<b2World> phys_world;
    b2Body* world_bounds = NULL;
    
    Squid squid;
    
    ofPtr<ofxUIScrollableCanvas> gui;
    
    // Matrices
    cv::Mat open_kernel;
    cv::Mat frame_full, frame, frame_gray;
    cv::Mat magnitude, angle, flow, flow_low, flow_low_prev, flow_high, flow_high_prev, flow_behind, flow_new;
    
    // Sizes
    double delta_t;
    double ratio;
    cv::Rect capture_roi;
    
    // Settings
    bool draw_debug = false;
    bool use_camera = false;
    bool resized = true;
        
    float flow_threshold_low = 0.1f;
    float flow_threshold_high = 0.5f;
    int flow_erosion_size = 5;
    float jaggy_spacing = 20.0f;
    float jaggy_offset = 10.0f;
    float fluid_motion_speed = 5.0f;
    float fluid_motion_radius = 10.0f;
};
