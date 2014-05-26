
#include "ofApp.h"
#include "constants.h"
#include "utilities.h"

using namespace ofxCv;

using std::cout;
using std::endl;

//--------------------------------------------------------------
void ofApp::setup()
{
    //
    // Generic openframeworks setup
    ofSetFrameRate(40);
    ofSetBackgroundAuto(false);
    ofSetMinMagFilters(GL_NEAREST, GL_NEAREST);
    ofClear(0, 0, 0, 255);
    ofEnableAlphaBlending();
    //
    // Set up Box2d
    setupPhysics();
    //
    // Setup GUI
    setupGUI();
    squid.setup(phys_world);
    //
    // Set up camera and video
    camera.initGrabber(kCaptureWidth, kCaptureHeight);
    video.loadMovie("videos/damrak/damrak_3.mov");
    video.setVolume(0);
    video.play();
    //
    // Set up optical flow
    open_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                            cv::Size(2 * flow_erosion_size + 1, 2 * flow_erosion_size + 1),
                                            cv::Point(flow_erosion_size, flow_erosion_size));
    opticalflow.setPyramidScale(0.5);
    opticalflow.setNumLevels(2);
    opticalflow.setWindowSize(7);
    opticalflow.setNumIterations(3);
    opticalflow.setPolyN(5);
    opticalflow.setPolySigma(1.2);
    //
    // Set up face detection
    objectfinder.setup("haarcascades/haarcascade_frontalface_alt2.xml");
//    objectfinder.setup("haarcascades/haarcascade_profileface.xml");
    objectfinder.setRescale(1.0); // Don't rescale internally because we'll feed it a small frame
    objectfinder.setMinNeighbors(2);
    objectfinder.setMultiScaleFactor(1.2);
    objectfinder.setFindBiggestObject(true);
    // Call resize to compute frame sizes the first time
    resize();
}

void ofApp::setupPhysics()
{
    b2Vec2 gravity(0.0f, 0.0f);
    phys_world = ofPtr<b2World> ( new b2World(gravity ) );
    // Set up the world bounds
    b2BodyDef boundsBodyDef;
    boundsBodyDef.position.Set(0, 0);
    world_bounds = phys_world.get()->CreateBody(&boundsBodyDef);
}

void ofApp::setupGUI()
{
    //
    // Set up control panel
    gui = ofPtr<ofxUIScrollableCanvas> (new ofxUIScrollableCanvas(0, 0, 200, ofGetHeight()));
    gui->setTheme(OFX_UI_THEME_HACKER);
    gui->setScrollAreaHeight(ofGetHeight());
    gui->setFontSize(OFX_UI_FONT_SMALL, 6);
    gui->addToggle("DEBUG", &draw_debug);
    gui->addToggle("CAMERA", &use_camera);
    gui->addSlider("FACE_SEARCH_WINDOW", 0.05, 1.0, 0.2);
    gui->addRangeSlider("FACE_SIZE", 0.02, 1.0, 0.05, 0.4);
    gui->addRangeSlider("FLOW_THRESHOLD", 0.0, 3.0, 0.1, 0.5);
    gui->addSlider("FLOW_EROSION_SIZE", 1, 7, 5);
    gui->addLabelButton("1080x480", false);
    gui->autoSizeToFitWidgets();
    gui->addMinimalSlider("SQUID_BODY_RADIUS", 10, 100, 40);
    gui->addMinimalSlider("SQUID_BODY_DENSITY", 0.1, 1.0, 0.2);
    // Save settings
    ofAddListener(gui->newGUIEvent, this, &ofApp::guiEvent);
    gui->loadSettings("settings.xml");
    gui->setPosition(ofGetWidth() - 200, 0);
}

void ofApp::resize()
{
    ratio = (float)ofGetWidth() / ofGetHeight();
    capture_roi = computeCenteredROI(frame_full, ratio);
    frame_scale = (double)ofGetWidth() / capture_roi.width;
    opticalflow.resetFlow();
    resizePhysics();
    resizeGUI();
}

void ofApp::resizePhysics()
{
    for (b2Fixture* f = world_bounds->GetFixtureList(); f; )
    {
        b2Fixture* fixtureToDestroy = f;
        f = f->GetNext();
        world_bounds->DestroyFixture( fixtureToDestroy );
    }
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

void ofApp::resizeGUI()
{
    gui->setPosition(ofGetWidth() - gui->getSRect()->width, 0);
    gui->setHeight(ofGetHeight());
    gui->setScrollAreaHeight(ofGetHeight());
}

//--------------------------------------------------------------
void ofApp::update()
{
    delta_t = ofGetLastFrameTime();
    doCapture();
    if (draw_debug)
    {
        ofClear(0, 0, 0, 255);
    }
    if (resized)
    {
        resize();
        resized = false;
    }
    updateFrame();
    if ((use_camera && camera.isFrameNew()) || video.isFrameNew() )
    {
        updateFlow();
        updateFinder();
    }
    squid.update(delta_t, flow_high, objectfinder, frame);
    // Update physics
    phys_world->Step(1.0f / kFrameRate, 6, 2);
}

void ofApp::doCapture()
{
    if (use_camera)
    {
        camera.update();
        frame_full = toCv(camera);
    }
    else
    {
        video.update();
        frame_full = toCv(video.getPixelsRef());
    }
}

void ofApp::updateFrame()
{
    cv::flip(frame_full(capture_roi), frame, 1);
    cv::cvtColor(frame, frame_gray, CV_BGR2GRAY);
    cv::pyrDown(frame_gray, frame_gray);
    cv::pyrDown(frame_gray, frame_gray);
}

void ofApp::updateFlow()
{
    opticalflow.calcOpticalFlow(frame_gray);
    flow = opticalflow.getFlow();
    // ofxCV wrapper returns a 1x1 flow image after the first optical flow computation.
    if (flow.cols == 1)
    {
        flow_low_prev = cv::Mat::zeros(frame_gray.rows, frame_gray.cols, CV_8U);
        flow_high_prev = cv::Mat::zeros(frame_gray.rows, frame_gray.cols, CV_8U);
        flow = cv::Mat::zeros(frame_gray.rows, frame_gray.cols, CV_32FC2);
    }
    std::vector<cv::Mat> xy(2);
    cv::split(flow, xy);
    cv::cartToPolar(xy[0], xy[1], magnitude, angle, true);
    //
    // Compute the low speed mask
    cv::threshold(magnitude, magnitude, flow_threshold_low, 1, cv::THRESH_TOZERO);
    flow_low = magnitude > 0;
    cv::erode(flow_low, flow_low, open_kernel);
    cv::dilate(flow_low, flow_low, open_kernel);
    //
    // Compute the high speed mask
    cv::threshold(magnitude, flow_high, flow_threshold_high, 1, cv::THRESH_TOZERO);
    flow_high = flow_high > 0; // & flow_low_prev > 0;
    cv::erode(flow_high, flow_high, open_kernel);
//    cv::dilate(flow_high, flow_high, open_kernel_small);
    contourfinder.findContours(flow_high);
    flow_behind = flow_high_prev & (255 - flow_high);
    flow_new = flow_high & ( 255 - flow_high_prev);
    std::swap(flow_low_prev, flow_low);
    std::swap(flow_high_prev, flow_high);
}

void ofApp::updateFinder()
{
    ofPoint squid_pos = squid.getPosition();
    // Only compute a width because the face search window is square.
    int face_search_width = MIN(frame.cols, frame.rows) * face_search_window;
    face_roi = cv::Rect(0, 0, face_search_width, face_search_width);
    face_roi += cv::Point(MAX(0, MIN(frame.cols - face_search_width, (squid_pos.x / frame_scale - face_search_width / 2))),
                          MAX(0, MIN(frame.rows - face_search_width, (squid_pos.y / frame_scale - face_search_width / 2))));
    // Face size is relative to frame, not face search window, so rescale and cap at 1.0
    objectfinder.setMinSizeScale(MIN(1.0, face_size_min / face_search_window));
    objectfinder.setMaxSizeScale(MIN(1.0, face_size_max / face_search_window));
    // The call below uses 2 arguments including a switch "preprocess" that
    // was added to ObjectFinder::update to disable the resize and BGR2GRAY calls.
    objectfinder.update(frame(face_roi), true);
}


//--------------------------------------------------------------
void ofApp::draw()
{
    ofSetColor(255, 255, 255, 255);
    ofxCv::drawMat(frame, 0, 0, ofGetWidth(), ofGetHeight());
//    drawParticles();
    if (draw_debug)
    {
        // Draw the optical flow maps
        ofSetColor(256, 0, 0, 196);
        ofEnableBlendMode(OF_BLENDMODE_ADD);
        ofxCv::drawMat(flow_high, 0, 0, ofGetWidth(), ofGetHeight());
        ofSetColor(224, 160, 58, 128);
        ofxCv::drawMat(flow_low, 0, 0, ofGetWidth(), ofGetHeight());
        ofDisableBlendMode();
        // Draw face search window
        ofRectangle face_roi_rect = toOf(face_roi);
        face_roi_rect.scale(frame_scale);
        face_roi_rect.x *= frame_scale;
        face_roi_rect.y *= frame_scale;
        ofPushStyle();
        ofSetLineWidth(2.0);
        ofSetColor(ofColor::orange);
        ofRect(face_roi_rect);
        ofDrawBitmapStringHighlight("face search window", face_roi_rect.getPosition() + kLabelOffset, ofColor::orange, ofColor::black);
        ofSetColor(255, 255, 255);
        ofRect(face_roi_rect.getPosition(), face_size_min * ofGetHeight(), face_size_min * ofGetHeight());
        ofRect(face_roi_rect.getPosition(), face_size_max * ofGetHeight(), face_size_max * ofGetHeight());
        // Draw detected faces
        for(int i = 0; i < objectfinder.size(); i++)
        {
            ofRectangle object = objectfinder.getObject(i);
            object.scale(frame_scale);
            object.translate(face_roi_rect.position);
            ofSetColor(ofColor::green);
            ofRect(object);
        }
        ofPopStyle();
    }
    squid.draw(draw_debug);
    ofDrawBitmapStringHighlight(ofToString(ofGetFrameRate()) + "fps", kLabelOffset);
    ofDrawBitmapStringHighlight("faces: " + ofToString(objectfinder.size()), kLabelOffset + ofPoint(0, 20));
}


void ofApp::exit()
{
    gui->saveSettings("settings.xml");
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key)
{
    switch (key)
    {
    case 'g':
        gui->toggleVisible();
        break;
    case 'd':
        draw_debug = !draw_debug;
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
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button)
{
}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h)
{
    resized = true;
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
    if (name == "CAMERA")
    {
        resized = true;
    }
    if (name == "FACE_SEARCH_WINDOW")
    {
        face_search_window = ((ofxUISlider*) e.widget)->getScaledValue();
    }
    if (name == "FACE_SIZE")
    {
        face_size_min = ((ofxUIRangeSlider*) e.widget)->getScaledValueLow();
        face_size_max = ((ofxUIRangeSlider*) e.widget)->getScaledValueHigh();
    }
    if (name == "FLOW_THRESHOLD")
    {
        flow_threshold_low = ((ofxUIRangeSlider*) e.widget)->getScaledValueLow();
        flow_threshold_high = ((ofxUIRangeSlider*) e.widget)->getScaledValueHigh();
    }
    if (name == "FLOW_EROSION_SIZE")
    {
        flow_erosion_size = (int)(((ofxUISlider*) e.widget)->getScaledValue());
    }
    if (name == "1080x480")
    {
        ofSetWindowShape(1080, 480);
        resized = true;
    }
    if (name == "SQUID_BODY_RADIUS")
    {
        squid.body_radius = (((ofxUISlider*) e.widget)->getScaledValue());
        squid.setup(phys_world);
    }
    if (name == "SQUID_BODY_DENSITY")
    {
        squid.body_density = (((ofxUISlider*) e.widget)->getScaledValue());
        squid.setup(phys_world);
    }
}
