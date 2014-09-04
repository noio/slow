
#include "ofApp.h"
#include "constants.h"
#include "utilities.h"
#include "ofxRemoteUIServer.h"

#include "math.h"

using namespace ofxCv;

ofApp::ofApp()
{
    phys_world = ofPtr<b2World> ( new b2World(b2Vec2(0.0f, 3.0f)) );
}

//--------------------------------------------------------------
void ofApp::setup()
{
    // Generic openframeworks setup
    ofSetFrameRate(kFrameRate);
    ofSetMinMagFilters(GL_NEAREST, GL_NEAREST);
    ofSetLogLevel(OF_LOG_NOTICE);
    // Videofeed
    VideoFeedWebcam* webcam = new VideoFeedWebcam();
    webcam->setup(0, capture_width, capture_height);
    webcam->setAspectRatio(ofGetWidth(), ofGetHeight());
    videofeed = ofPtr<VideoFeed>(webcam);
    flowcam.setup(160);
    // Visualizer
    visualizer.setup(&flowcam);
    // Ambient Player
    sounds.setup();
    // Highscore table
    highscores.setup(ofGetWidth() / 10, &visualizer);
    // Set up Box2d
    setupPhysics();
    // Squid setup
    squid.setup(phys_world, &flowcam, &visualizer, &sounds, &highscores);
    // Instructions
    instructions.setup(&squid);
    // Gui Setup
    setupGUI();
}

void ofApp::setupGUI()
{
    OFX_REMOTEUI_SERVER_SETUP(44040); //start server
    OFX_REMOTEUI_SERVER_NEW_GROUP("Global");
    OFX_REMOTEUI_SERVER_SHARE_PARAM(draw_debug);
    // TODO OFX_REMOTEUI_SERVER_SHARE_PARAM(flip)
    // TODO camera res
    // TODO resolution & position
    
    OFX_REMOTEUI_SERVER_NEW_GROUP("Flow");
    OFX_REMOTEUI_SERVER_SHARE_PARAM(flowcam.flow_erosion_size, 1, 11);
    OFX_REMOTEUI_SERVER_SHARE_PARAM(flowcam.flow_threshold_low, 0.0f, 1.0f);
    OFX_REMOTEUI_SERVER_SHARE_PARAM(flowcam.flow_threshold_high, 0.0f, 2.0f);
    
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

}

void ofApp::setupPhysics()
{
    if (world_bounds != NULL) {
        world_bounds->GetWorld()->DestroyBody(world_bounds);
    }

    // Set up the world bounds
    b2BodyDef boundsBodyDef;
    boundsBodyDef.position.Set(0, 0);
    world_bounds = phys_world->CreateBody(&boundsBodyDef);
    b2EdgeShape shape;
    b2AABB rec = ofToB2(ofRectangle(-kGameSizePadding, -kGameSizePadding,
                                    (ofGetWidth() + kGameSizePadding * 2), (ofGetHeight() + kGameSizePadding * 2)));
    //right wall
    shape.Set(b2Vec2(rec.upperBound.x, rec.lowerBound.y), b2Vec2(rec.upperBound.x, rec.upperBound.y));
    world_bounds->CreateFixture(&shape, 0.0f);
    //left wall
    shape.Set(b2Vec2(rec.lowerBound.x, rec.lowerBound.y), b2Vec2(rec.lowerBound.x, rec.upperBound.y));
    world_bounds->CreateFixture(&shape, 0.0f);
    // top wall
    shape.Set(b2Vec2(rec.lowerBound.x, rec.lowerBound.y), b2Vec2(rec.upperBound.x, rec.lowerBound.y));
    world_bounds->CreateFixture(&shape, 0.0f);
    // bottom wall
    shape.Set(b2Vec2(rec.lowerBound.x, rec.upperBound.y), b2Vec2(rec.upperBound.x, rec.upperBound.y));
    world_bounds->CreateFixture(&shape, 0.0f);
}



//--------------------------------------------------------------
void ofApp::update()
{
    delta_t = ofGetLastFrameTime();
    timeout -= delta_t;
    if (timeout < 0){
        ofLogNotice("ofApp") << "timeout after ";
        std::exit(0);
    }
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
    // TODO HANDLE DIS
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg)
{
}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo)
{
}
