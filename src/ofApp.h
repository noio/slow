#pragma once

#include "videofeed.h"
#include "flowcam.h"
#include "motionvisualizer.h"
#include "squid.h"
#include "highscoretable.h"
#include "instructions.h"

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxUI.h"
#include "ofxFluid.h"
#include "ofxDelaunay.h"
#include "Box2D/Box2D.h"

#include <vector>

using std::vector;
using std::string;

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
    
    void setTimeoutFromGUI();
    void setWindowPositionFromGUI();
    void setWindowSizeFromGUI();
    void unfocusAllTextInputs(ofxUITextInput * except);

    // Components
    ofPtr<b2World> phys_world;
    b2Body* world_bounds = NULL;
    ofPtr<VideoFeed> videofeed;
    FlowCam flowcam;
    MotionVisualizer visualizer;
    AmbientPlayer sounds;
    Squid squid;
    HighscoreTable highscores;
    Instructions instructions;
    
    double delta_t;
    float timeout = 36000;
    
    // Settings
    bool draw_debug = false;
    int capture_width = 1280;
    int capture_height = 720;
};
