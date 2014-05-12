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

    void updateFrame();
    void updateFlow();
    void updateFinder();
    void updateMotionEffect();
    void updateFluid();
    void updateParticles();

    void drawParticles();

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

    ofVideoPlayer video;
    ofVideoGrabber camera;

    FluidSolver fluid;
    ofTexture fluid_texture;

    ParticleSystem particles;

    ofxCv::FlowFarneback opticalflow;
    ofxCv::ContourFinder contourfinder;
    ofxCv::ObjectFinder objectfinder;
    ofxDelaunay triangulator;
    
    ofPtr<b2World> phys_world;
    Squid squid;
    
    ofxUISuperCanvas* gui;
    
    cv::Mat open_kernel;
    cv::Mat frame, frame_gray;
    cv::Mat magnitude, angle, flow, flow_low, flow_low_prev, flow_high, flow_high_prev, flow_behind, flow_new;
    
    double delta_t;
    cv::Rect face_roi;
    
    // Settings
    bool draw_debug = false;
    bool use_camera = false;
    float face_search_window = 0.2;
    float face_max_size = 0.6;  // This is relative to the full frame, not the face search window
    float face_min_size = 0.05; // This is relative to the full frame, not the face search window
};
