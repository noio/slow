#pragma once

#include "videofeed.h"
#include "flowcam.h"
#include "motionvisualizer.h"
#include "squid.h"
#include "highscoretable.h"
#include "instructions.h"

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxDelaunay.h"
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
    
    static void remoteUICallback(RemoteUIServerCallBackArg arg);
    
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
    
    bool setup_done = false;
    
    // Settings
    bool draw_debug = false;
    int window_x = 20;
    int window_y = 100;
    int window_width = 896;
    int window_height = 288;
    VideoFeedWebcamResolution capture_res = WEBCAM_RES_720;
    
    // Mirrored settings
    float squid_scale = 1.0;
};
