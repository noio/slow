
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
    if (!gui_initialized){
        // Clear control panel
        gui->removeWidgets();
        textInputs.clear();
        // Set it up
        gui->setGlobalCanvasWidth(kGUIWidth);
        gui->setTheme(OFX_UI_THEME_MINBLACK);
        gui->setScrollAreaHeight(ofGetHeight());
        gui->setFontSize(OFX_UI_FONT_SMALL, 6);
        gui->addLabelButton("RESET", false);
        defaultTimeoutTextInput = gui->addTextInput("DEFAULT_TIMEOUT", "3600");
        defaultTimeoutTextInput->setAutoClear(false);
        textInputs.push_back(defaultTimeoutTextInput);
        gui->addNumberDialer("TIMEOUT", 0, 36000, &timeout, 0);
        // ----------
        gui->addLabel("CAMERA");
        gui->addToggle("DEBUG", &draw_debug);
        gui->addSlider("ZOOM", 1.0, 2.0, 1.0);
        gui->addIntSlider("FLIP", -1, 2, 1);
        gui->setGlobalCanvasWidth(kGUIWidth/2);
        gui->addLabelButton("720p", false);
        gui->setWidgetPosition(OFX_UI_WIDGET_POSITION_RIGHT);
        gui->addLabelButton("1080p", false);
        gui->setWidgetPosition(OFX_UI_WIDGET_POSITION_DOWN);
        gui->setGlobalCanvasWidth(kGUIWidth);
        // -----------
        gui->addLabel("RESOLUTION");
        gui->addLabelButton("768x288", false);
        gui->setGlobalCanvasWidth(kGUIWidth/2);
        windowWTextInput = gui->addTextInput("WINDOW_W", ofToString(ofGetWidth()));
        windowWTextInput->setAutoClear(false);
        textInputs.push_back(windowWTextInput);
        gui->setWidgetPosition(OFX_UI_WIDGET_POSITION_RIGHT);
        windowHTextInput = gui->addTextInput("WINDOW_H", ofToString(ofGetHeight()));
        gui->setWidgetPosition(OFX_UI_WIDGET_POSITION_DOWN);
        windowHTextInput->setAutoClear(false);
        textInputs.push_back(windowHTextInput);
        gui->setGlobalCanvasWidth(kGUIWidth);
        gui->addLabel("WINDOW POSITION");
        gui->setGlobalCanvasWidth(kGUIWidth/2);
        windowXTextInput = gui->addTextInput("WINDOW_X", "0");
        windowXTextInput->setAutoClear(false);
        textInputs.push_back(windowXTextInput);
        gui->setWidgetPosition(OFX_UI_WIDGET_POSITION_RIGHT);
        windowYTextInput = gui->addTextInput("WINDOW_Y", "0");
        gui->setWidgetPosition(OFX_UI_WIDGET_POSITION_DOWN);
        windowYTextInput->setAutoClear(false);
        textInputs.push_back(windowYTextInput);
        gui->setGlobalCanvasWidth(kGUIWidth);
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
        gui->loadSettings("settings.xml");
        gui_initialized = true;
    }
    // Position the GUI
    gui->setWidth(kGUIWidth);
    gui->setPosition(ofGetWidth() - kGUIWidth, 0);
    gui->setHeight(ofGetHeight());
    gui->setScrollAreaHeight(ofGetHeight());
    gui->autoSizeToFitWidgets();
    // Load settings

    gui->setVisible(false);
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
    ofLogNotice("ofApp") << "resized " << w << "x" << h;
    need_setup = true;
    windowWTextInput->setTextString(ofToString(w));
    windowHTextInput->setTextString(ofToString(h));
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
    if (kind == OFX_UI_WIDGET_TEXTINPUT){
        ofxUITextInput *ti = (ofxUITextInput *) e.widget;
        if (ti->getInputTriggerType() == OFX_UI_TEXTINPUT_ON_FOCUS){
            unfocusAllTextInputs(ti);
        }
    }

    if(name == "DEFAULT_TIMEOUT")
    {
        ofxUITextInput *ti = (ofxUITextInput *) e.widget;
        if(ti->getInputTriggerType() == OFX_UI_TEXTINPUT_ON_ENTER)
        {
            setTimeoutFromGUI();
        }
    }
    
    if(name == "WINDOW_X")
    {
        ofxUITextInput *ti = (ofxUITextInput *) e.widget;
        if(ti->getInputTriggerType() == OFX_UI_TEXTINPUT_ON_ENTER)
        {
            setWindowPositionFromGUI();
        }
    }
    
    if(name == "WINDOW_Y")
    {
        ofxUITextInput *ti = (ofxUITextInput *) e.widget;
        if(ti->getInputTriggerType() == OFX_UI_TEXTINPUT_ON_ENTER)
        {
            setWindowPositionFromGUI();
        }
    }
    
    if(name == "WINDOW_W")
    {
        ofxUITextInput *ti = (ofxUITextInput *) e.widget;
        if(ti->getInputTriggerType() == OFX_UI_TEXTINPUT_ON_ENTER)
        {
            setWindowSizeFromGUI();
        }
    }
    
    if(name == "WINDOW_H")
    {
        ofxUITextInput *ti = (ofxUITextInput *) e.widget;
        if(ti->getInputTriggerType() == OFX_UI_TEXTINPUT_ON_ENTER)
        {
            setWindowSizeFromGUI();
        }
    }

}

void ofApp::setTimeoutFromGUI(){
    timeout = ofToInt(defaultTimeoutTextInput->getTextString());
}

void ofApp::setWindowPositionFromGUI(){
    ofSetWindowPosition(ofToInt(windowXTextInput->getTextString()), ofToInt(windowYTextInput->getTextString()));
}

void ofApp::setWindowSizeFromGUI(){
    ofSetWindowShape(ofToInt(windowWTextInput->getTextString()), ofToInt(windowHTextInput->getTextString()));
}

void ofApp::unfocusAllTextInputs(ofxUITextInput* except){
    for (int i = 0; i < textInputs.size(); i ++){
        if (except != textInputs[i]){
            textInputs[i]->setFocus(false);
        }
    }
}
