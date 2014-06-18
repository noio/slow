
#include "ofApp.h"
#include "constants.h"
#include "utilities.h"

#include "math.h"

using namespace ofxCv;

using std::cout;
using std::endl;

ofApp::ofApp()
{
    gui = ofPtr<ofxUIScrollableCanvas> (new ofxUIScrollableCanvas(0, 0, 200, ofGetHeight()));
    phys_world = ofPtr<b2World> ( new b2World(b2Vec2(0.0f, 3.0f)) );
}

//--------------------------------------------------------------
void ofApp::setup()
{
    // Generic openframeworks setup
    ofSetFrameRate(kFrameRate);
    ofSetBackgroundAuto(false);
    ofSetMinMagFilters(GL_NEAREST, GL_NEAREST);
    ofClear(0, 0, 0, 255);
    ofEnableAlphaBlending();
    ofSetLogLevel(OF_LOG_NOTICE);
    // Flow camera
    if (flowcam.isThreadRunning()){
        flowcam.waitForThread(true);
    }
    flowcam.setup(capture_width, capture_height, ofGetWidth(), ofGetHeight(), 1.0);
//    flowcam.startThread(true,true);
    // Visualizer
    visualizer.setup(&flowcam);
    // Highscore table
    highscores.setup(ofGetWidth() / 10, &visualizer);
    // Set up Box2d
    setupPhysics();
    // Squid setup
    squid.setup(phys_world, &flowcam, &visualizer, &highscores);
    // Instructions
    instructions.setup(&squid);
    // Gui Setup
    setupGUI();
    need_setup = false;
//    flowcam.startThread(true, false);
}

void ofApp::setupGUI()
{
    int w = 210;
    // Set up control panel
    gui->removeWidgets();
    gui->setGlobalCanvasWidth(w);
    gui->setTheme(OFX_UI_THEME_MINBLACK);
    gui->setScrollAreaHeight(ofGetHeight());
    gui->setFontSize(OFX_UI_FONT_SMALL, 6);
    gui->addLabelButton("RESET", false);
    defaultTimeoutTextInput = gui->addTextInput("DEFAULT_TIMEOUT", "3600");
    defaultTimeoutTextInput->setAutoClear(false);
    gui->addNumberDialer("TIMEOUT", 0, 36000, &timeout, 0);
    // ----------
    gui->addLabel("CAMERA");
    gui->addToggle("DEBUG", &draw_debug);
    gui->addSlider("ZOOM", 1.0, 2.0, 1.0);
    gui->addIntSlider("FLIP", -1, 2, 1);
    gui->setGlobalCanvasWidth(w/2);
	gui->addLabelButton("720p", false);
    gui->setWidgetPosition(OFX_UI_WIDGET_POSITION_RIGHT);
    gui->addLabelButton("1080p", false);
    gui->setWidgetPosition(OFX_UI_WIDGET_POSITION_DOWN);
    gui->setGlobalCanvasWidth(w);
    // -----------
    gui->addLabel("RESOLUTION");
    gui->addLabelButton("1080x480", false);
    gui->addLabelButton("768x288", false);
    gui->addLabel("WINDOW POSITION");
    gui->setGlobalCanvasWidth(w/2);
    windowXTextInput = gui->addTextInput("WINDOW_X", "0");
    windowXTextInput->setAutoClear(false);
    gui->setWidgetPosition(OFX_UI_WIDGET_POSITION_RIGHT);
    windowYTextInput = gui->addTextInput("WINDOW_Y", "0");
    gui->setWidgetPosition(OFX_UI_WIDGET_POSITION_DOWN);
    windowYTextInput->setAutoClear(false);
    gui->setGlobalCanvasWidth(w);
    // ----------
    gui->addLabel("OPTICAL FLOW");
    gui->addRangeSlider("FLOW_THRESHOLD", 0.0, 3.0, 0.1, 0.5);
    gui->addIntSlider("FLOW_EROSION_SIZE", 1, 11, 5);
    // ----------
    gui->addLabel("SQUISHY");
    gui->addSlider("SQUID_SCALE", 0.5f, 3.0f, 1.9f);
    gui->addSlider("FACE_SEARCH_WINDOW", 0.05, 1.0, 0.2);
    gui->addRangeSlider("FACE_SIZE", 0.02, 1.0, 0.05, 0.4);
    gui->addSlider("PUSH_FORCE", 20.0f, 100.0f, &squid.push_force);
    gui->addSlider("LOCAL_AREA", 80.0f, 200.0f, &squid.local_area_radius);
    gui->addSlider("CORE_AREA", 20.0f, 80.0f, &squid.core_area_radius);
    gui->addSlider("MAX_LOCAL_FLOW", 0.0f, 1.0f, &squid.local_flow_max);
    gui->addSlider("MAX_CORE_FLOW", 0.0, 1.0f, &squid.core_flow_max);
    // ----------
    gui->addLabel("VISUALIZER");
    gui->addIntSlider("HUE", 0, 255, &visualizer.trail_hue);
    gui->addIntSlider("HUE_RANGE", 0, 255, &visualizer.trail_hue_range);
    gui->addSlider("ALPHA", 0.0, 4.0, &visualizer.trail_alpha_mtp);
    // Size
    ofAddListener(gui->newGUIEvent, this, &ofApp::guiEvent);
    // Position the GUI
    gui->setWidth(w);
    gui->setPosition(ofGetWidth() - gui->getSRect()->width, 0);
    gui->setHeight(ofGetHeight());
    gui->setScrollAreaHeight(ofGetHeight());
    gui->autoSizeToFitWidgets();
    // Load settings
    gui->loadSettings("settings.xml");
    // Set some other things
    setTimeoutFromGUI();
    setWindowPositionFromGUI();
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
    if (need_setup) {
        setup();
    }

    delta_t = ofGetLastFrameTime();
    timeout -= delta_t;
    if (timeout < 0){
        ofLogNotice("ofApp") << "timeout after ";
        std::exit(0);
    }
    flowcam.update();
    squid.update(delta_t);
    highscores.update(delta_t);
    visualizer.update(delta_t);
    instructions.update(delta_t);
    phys_world->Step(delta_t, 6, 2);
}


//--------------------------------------------------------------
void ofApp::draw()
{
    visualizer.draw();
    highscores.draw();
    instructions.draw();
    squid.draw(draw_debug);

    if (draw_debug) {
        flowcam.drawDebug();
        ofDrawBitmapStringHighlight(ofToString(ofGetFrameRate()) + "fps", kLabelOffset);
    }
}


void ofApp::exit()
{
    gui->saveSettings("settings.xml");
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key)
{
    switch (key) {
        case 'g':
            gui->toggleVisible();
            break;

        case 'd':
            draw_debug = !draw_debug;
            break;

        case 'h':
            instructions.play();
            break;

        case ' ':
            ofSleepMillis(20000);
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
    need_setup = true;
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg)
{
}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo)
{
}

//--------------------------------------------------------------
void ofApp::guiEvent(ofxUIEventArgs& e)
{
    string name = e.widget->getName();
    int kind = e.widget->getKind();

    if (name == "RESET") {
        gui->saveSettings("settings.xml");
        need_setup = true;
    }
    
    if (name == "1080p"){
        flowcam.setCaptureSize(1920, 1080);
    }
    if (name == "720p"){
        flowcam.setCaptureSize(1280, 720);
    }

    if (name == "FACE_SEARCH_WINDOW") {
        squid.face_search_window = ((ofxUISlider*) e.widget)->getScaledValue();
    }

    if (name == "FACE_SIZE") {
        squid.face_size_min = ((ofxUIRangeSlider*) e.widget)->getScaledValueLow();
        squid.face_size_max = ((ofxUIRangeSlider*) e.widget)->getScaledValueHigh();
    }

    if (name == "FLOW_THRESHOLD") {
        flowcam.setFlowThreshold(((ofxUIRangeSlider*) e.widget)->getScaledValueLow(), ((ofxUIRangeSlider*) e.widget)->getScaledValueHigh());
    }

    if (name == "FLOW_EROSION_SIZE") {
        flowcam.setFlowErosionSize((int)(((ofxUIIntSlider*) e.widget)->getScaledValue()));
    }

    if (name == "SQUID_SCALE") {
        squid.setScale(((ofxUISlider*) e.widget)->getScaledValue());
    }

    if (name == "ZOOM") {
        flowcam.setZoom(((ofxUISlider*) e.widget)->getScaledValue());
    }

    if (name == "1080x480") {
        ofSetWindowShape(1080, 480);
        need_setup = true;
    }

    if (name == "768x288") {
        ofSetWindowShape(768, 288);
        need_setup = true;
    }
    
    if (name == "FLIP"){
        flowcam.setFlip(((ofxUIIntSlider*) e.widget)->getScaledValue());
    }
    // Text fields
    if(name == "DEFAULT_TIMEOUT")
    {
        ofxUITextInput *ti = (ofxUITextInput *) e.widget;
        if(ti->getInputTriggerType() == OFX_UI_TEXTINPUT_ON_ENTER || ti->getInputTriggerType() == OFX_UI_TEXTINPUT_ON_UNFOCUS)
        {
            setTimeoutFromGUI();
        }
    }
    
    if(name == "WINDOW_X")
    {
        ofxUITextInput *ti = (ofxUITextInput *) e.widget;
        if(ti->getInputTriggerType() == OFX_UI_TEXTINPUT_ON_ENTER || ti->getInputTriggerType() == OFX_UI_TEXTINPUT_ON_UNFOCUS)
        {
            setWindowPositionFromGUI();
        }
    }
    
    if(name == "WINDOW_Y")
    {
        ofxUITextInput *ti = (ofxUITextInput *) e.widget;
        if(ti->getInputTriggerType() == OFX_UI_TEXTINPUT_ON_ENTER || ti->getInputTriggerType() == OFX_UI_TEXTINPUT_ON_UNFOCUS)
        {
            setWindowPositionFromGUI();
        }
    }

}

void ofApp::setTimeoutFromGUI(){
    timeout = ofToInt(defaultTimeoutTextInput->getTextString());
}

void ofApp::setWindowPositionFromGUI(){
    ofSetWindowPosition(ofToInt(windowXTextInput->getTextString()), ofToInt(windowYTextInput->getTextString()));
}