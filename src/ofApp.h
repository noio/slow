#pragma once

#include "Squid.h"
#include "Fluid.h"
#include "ParticleSystem.h"

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxUI.h"
#include "ofxDelaunay.h"
#include "Box2D/Box2D.h"


class ofApp : public ofBaseApp
{

public:

    void setup();
    void update();
    void draw();
    void exit();
    
    void setupPhysics();
    void setupGUI();
    
    void resize();
    void resizePhysics();
    void resizeGUI();
    
    void doCapture();

    void updateFrame();
    void updateFlow();
    void updateFinder();

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

    ofxCv::FlowFarneback opticalflow;
    ofxCv::ContourFinder contourfinder;
    ofxCv::ObjectFinder objectfinder;
    
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
    cv::Rect face_roi;
    cv::Rect capture_roi;
    double frame_scale;
    
    // Settings
    bool draw_debug = false;
    bool source_camera = false;
    bool resized = true;
    
    float face_search_window = 0.2;
    float face_size_min = 0.6;  // This is relative to the full frame, not the face search window
    float face_size_max = 0.05; // This is relative to the full frame, not the face search window
    
    float flow_threshold_low = 0.1f;
    float flow_threshold_high = 0.5f;
    int flow_erosion_size = 5;
};
