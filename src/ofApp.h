#pragma once

#include "motionvisualizer.h"
#include "squid.h"
#include "highscoretable.h"
#include "instructions.h"

#include "ofxDropstuff.h"
#include "ofMain.h"
#include "ofxCv.h"
#include "ofxRemoteUIServer.h"
#include "Box2D/Box2D.h"

#include <vector>

using std::vector;
using std::string;

class ofApp : public ofBaseApp
{

public:

    void setup();
    void update();
    void draw();
    void exit();

    void setupGUI();
    void createPhysicsBounds();

    void keyPressed(int key);
    void keyReleased(int key);
    void mouseMoved(int x, int y );
    void mouseDragged(int x, int y, int button);
    void mousePressed(int x, int y, int button);
    void mouseReleased(int x, int y, int button);
    void windowResized(int w, int h);
    void dragEvent(ofDragInfo dragInfo);
    void gotMessage(ofMessage msg);

    // Components
    ofPtr<b2World> phys_world;
    b2Body* world_bounds = NULL;
    ofPtr<ofxDS::VideoFeed> videofeed;
    ofxDS::FlowCam flowcam;
    MotionVisualizer visualizer;
    AmbientPlayer sounds;
    Squid squid;
    HighscoreTable highscores;
    Instructions instructions;

    double delta_t;

    // Settings
    bool draw_debug = false;
    bool use_imagefeed = false;
    std::string imagefeed_address = "http://localhost:1338/color";
    int window_x = 20;
    int window_y = 100;
    int window_width = 896;
    int window_height = 288;
    ofxDS::VideoFeedWebcamResolution capture_res = ofxDS::WEBCAM_RES_720;

    // Mirrored settings
    float squid_scale = 1.0;
};
