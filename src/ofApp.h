#pragma once

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxUI.h"
#include "ofxDelaunay.h"
#include "ofxBox2d.h"

#include "Fluid.h"
#include "ParticleSystem.h"
#include "Squid.h"

class ofApp : public ofBaseApp
{

public:
    ofApp() : gui("EDIT")
    { };

    void setup();
    void update();
    void draw();
    void exit();

    void updateFlow();
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

    ofxUISuperCanvas gui;
    bool draw_debug = false;

    cv::Mat open_kernel;
    cv::Mat frame, frame_gray;
    cv::Mat magnitude, angle, flow, flow_low, flow_low_prev, flow_high, flow_high_prev, flow_behind, flow_new;

    ofVideoPlayer video;
    ofVideoGrabber camera;

    FluidSolver fluid;
    ofTexture fluid_texture;

    ParticleSystem particles;

    ofxBox2d box2d;
    Squid squid;

    ofxCv::FlowFarneback opticalflow;
    ofxCv::ContourFinder contourfinder;
    ofxDelaunay triangulator;
    
    double delta_t;
};
