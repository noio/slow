
#include "ofApp.h"
#include "constants.h"
#include "utilities.h"


#include "math.h"

using namespace ofxCv;
using namespace ofxDS;

//--------------------------------------------------------------
void ofApp::setup()
{
    // Generic openframeworks setup
    ofSetFrameRate(kFrameRate);
    ofSetMinMagFilters(GL_NEAREST, GL_NEAREST);
    ofSetLogLevel(OF_LOG_NOTICE);
    // Gui Setup
    setupGUI();
    // Window setup
    ofSetWindowPosition(window_x, window_y);
    ofSetWindowShape(window_width, window_height);
    // Videofeed
    if (use_imagefeed){
        VideoFeedImageURL* feed = new VideoFeedImageURL();
        feed->setup(imagefeed_address);
        feed->setAspectRatio(ofGetWidth(), ofGetHeight());
        videofeed = ofPtr<VideoFeed>(feed);
    }
    else
    {
        VideoFeedWebcam* webcam = new VideoFeedWebcam();
        webcam->setup(0, capture_res);
        webcam->setAspectRatio(ofGetWidth(), ofGetHeight());
        videofeed = ofPtr<VideoFeed>(webcam);
    }
    flowcam.setup(160);
    // Visualizer
    visualizer.setup(&flowcam);
    // Ambient Player
    sounds.setup();
    // Highscore table
    highscores.setup(ofGetWidth() / 10, &visualizer);
    // Set up Box2d
    phys_world = ofPtr<b2World> ( new b2World(b2Vec2(0.0f, 3.0f)) );
    // Squid setup
    squid.setup(squid_scale, phys_world, &flowcam, &visualizer, &sounds, &highscores);
    // Instructions
    instructions.setup(&squid);
}

void ofApp::setupGUI()
{
    OFX_REMOTEUI_SERVER_SETUP(44040); //start server
    
    OFX_REMOTEUI_SERVER_NEW_GROUP("Global");
    OFX_REMOTEUI_SERVER_SHARE_PARAM(draw_debug);
    //
    OFX_REMOTEUI_SERVER_NEW_GROUP("Reboot");
    OFX_REMOTEUI_SERVER_SHARE_PARAM(squid_scale, 0.2, 4.0);
    OFX_REMOTEUI_SERVER_SHARE_PARAM(window_x, 0, ofGetScreenWidth() / 2);
    OFX_REMOTEUI_SERVER_SHARE_PARAM(window_y, 0, ofGetScreenHeight() / 2);
    OFX_REMOTEUI_SERVER_SHARE_PARAM(window_width, 320, 1024);
    OFX_REMOTEUI_SERVER_SHARE_PARAM(window_height, 240, 768);
    vector<string> resolutions;
    resolutions.push_back("640x480");
    resolutions.push_back("720p");
    resolutions.push_back("1080p");
    OFX_REMOTEUI_SERVER_SHARE_ENUM_PARAM(capture_res, 0, 2, resolutions);
    //
    OFX_REMOTEUI_SERVER_NEW_GROUP("Flow");
    OFX_REMOTEUI_SERVER_SHARE_PARAM(flowcam.flow_erosion_size, 1, 11);
    OFX_REMOTEUI_SERVER_SHARE_PARAM(flowcam.flow_threshold_low, 0.0f, 1.0f);
    OFX_REMOTEUI_SERVER_SHARE_PARAM(flowcam.flow_threshold_high, 0.0f, 2.0f);
    //
    OFX_REMOTEUI_SERVER_NEW_GROUP("Squishy");
    // TODO callback for scale
    OFX_REMOTEUI_SERVER_SHARE_PARAM(squid.face_search_window, 0.1, 1.0);
    OFX_REMOTEUI_SERVER_SHARE_PARAM(squid.face_size_min, 0.05, 0.2);
    OFX_REMOTEUI_SERVER_SHARE_PARAM(squid.face_size_max, 0.1, 1.0);
    OFX_REMOTEUI_SERVER_SHARE_PARAM(squid.push_force, 20, 100);
    OFX_REMOTEUI_SERVER_SHARE_PARAM(squid.local_area_radius, 80, 200);
    OFX_REMOTEUI_SERVER_SHARE_PARAM(squid.core_area_radius, 20, 80);
    OFX_REMOTEUI_SERVER_SHARE_PARAM(squid.local_flow_max, 0, 0.5);
    OFX_REMOTEUI_SERVER_SHARE_PARAM(squid.core_flow_max, 0, 0.5);
    
    OFX_REMOTEUI_SERVER_NEW_GROUP("Visualizer");
    OFX_REMOTEUI_SERVER_SHARE_PARAM(visualizer.trail_hue, 0, 255);
    OFX_REMOTEUI_SERVER_SHARE_PARAM(visualizer.trail_hue_range, 0, 255);
    OFX_REMOTEUI_SERVER_SHARE_PARAM(visualizer.trail_alpha_mtp, 0, 4.0);

    OFX_REMOTEUI_SERVER_LOAD_FROM_XML();

}

//--------------------------------------------------------------
void ofApp::update()
{
    delta_t = ofGetLastFrameTime();

    cv::Mat frame;
    if (videofeed->getFrame(frame))
    {
        flowcam.update(frame);
    }
    squid.update(delta_t, frame);
    highscores.update(delta_t);
    visualizer.update(delta_t);
    sounds.update(delta_t);
    instructions.update(delta_t);
    phys_world->Step(delta_t, 6, 2);
}


//--------------------------------------------------------------
void ofApp::draw()
{
    //TODO Set color
    videofeed->draw(0, 0, ofGetWidth(), ofGetHeight());
    visualizer.draw();
    highscores.draw();
    instructions.draw();
    squid.draw(draw_debug);

    if (draw_debug) {
        flowcam.drawDebug();
        ofDrawBitmapStringHighlight(ofToString(ofGetFrameRate(),0) + "fps (r99)", kLabelOffset);
        ofDrawBitmapStringHighlight("[d]ebug view \n[h]elp \n[ ] crash \n[i] sound on \n[o] sound off \n[r]eset", kLabelOffset + ofPoint(0,20));
    }
}


void ofApp::exit()
{
    videofeed.reset();
    window_x = ofGetWindowPositionX();
    window_y = ofGetWindowPositionY();
    ofLogVerbose("ofApp") << "exiting";
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key)
{
    switch (key) {
        case 'd':
            draw_debug = !draw_debug;
            break;

        case 'h':
            instructions.play();
            break;

        case ' ':
            ofSleepMillis(20000);
            break;
            
        case 'i':
            sounds.setSoundsOn(true);
            break;
            
        case 'o':
            sounds.setSoundsOn(false);
            break;
            
        default:
            break;
    }
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key)
{
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y )
{
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button)
{
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button)
{
    squid.stayAtPoint(ofPoint(x, y), 10);
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button)
{
}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h)
{
    ofLogNotice("ofApp") << "resized " << w << "x" << h;
    window_width = w;
    window_height = h;
    videofeed->setAspectRatio(w, h);
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg)
{
}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo)
{
}

