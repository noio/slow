#pragma once

#include "squid.h"
#include "flowcam.h"
#include "motionvisualizer.h"
#include "highscoretable.h"

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxUI.h"
#include "ofxFluid.h"
#include "ofxDelaunay.h"
#include "Box2D/Box2D.h"


class ofApp : public ofBaseApp
{

public:
    ofApp();
    
    void setup();
    void update();
    void draw();
    void exit();
    
    void setupPhysics();
    void setupGUI();

    void updateFrame();
    void updateFlow();
    
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
    ofImage logo_im;
    ofPtr<b2World> phys_world;
    b2Body* world_bounds = NULL;
    FlowCam flowcam;
    MotionVisualizer visualizer;
    HighscoreTable highscores;
    Squid squid;
    
    ofPtr<ofxUIScrollableCanvas> gui;
    
    double delta_t;
    
    bool need_setup = false;
    
    // Settings
    bool draw_debug = false;
        
    float jaggy_spacing = 20.0f;
    float jaggy_offset = 10.0f;
    float fluid_motion_speed = 5.0f;
    float fluid_motion_radius = 10.0f;
};
