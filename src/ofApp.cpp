
#include "ofApp.h"
#include "constants.h"
#include "utilities.h"

#include "math.h"

using namespace ofxCv;

using std::cout;
using std::endl;

//--------------------------------------------------------------
void ofApp::setup()
{
    // Generic openframeworks setup
    ofSetFrameRate(40);
    ofSetBackgroundAuto(false);
    ofSetMinMagFilters(GL_NEAREST, GL_NEAREST);
    ofClear(0, 0, 0, 255);
    ofEnableAlphaBlending();
    contourfinder.setSimplify(true);
    contourfinder.setMinArea(80);
    // Load textures
    motion_texture_a.loadImage("assets/motion_texture_a.png");
    motion_texture_a.getTextureReference().setTextureWrap(GL_REPEAT, GL_REPEAT);
    // Setup GUI
    setupGUI();

    // Use an external camera if one is connected
    if (camera.listDevices().size() > 1)
    {
        camera.setDeviceID(1);
    }

    camera.initGrabber(kCaptureWidth, kCaptureHeight);
    video.loadMovie("videos/damrak/damrak_3.mov");
    video.setVolume(0);
    video.play();
    // Reset / start
    reset();
}

void ofApp::reset()
{
    //
    // Set up Box2d
    setupPhysics();
    squid.setup(phys_world);
    doCapture(); // Get a single frame so we have the resolution
    ratio = (float)ofGetWidth() / ofGetHeight();
    capture_roi = computeCenteredROI(frame_full, ratio);
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
    opticalflow.resetFlow();
    // Set up fluid
    fluid.allocate(ofGetWidth(), ofGetHeight(), kFluidScale);
    fluid.dissipation = 0.99;
    fluid.velocityDissipation = 0.99;
    fluid.setGravity(ofVec2f(0.0, 0.0));
    // Position the GUI
    gui->setPosition(ofGetWidth() - gui->getSRect()->width, 0);
    gui->setHeight(ofGetHeight());
    gui->setScrollAreaHeight(ofGetHeight());
}

void ofApp::setupGUI()
{
    //
    // Set up control panel
    gui = ofPtr<ofxUIScrollableCanvas> (new ofxUIScrollableCanvas(0, 0, 200, ofGetHeight()));
    gui->setTheme(OFX_UI_THEME_HACKER);
    gui->setScrollAreaHeight(ofGetHeight());
    gui->setFontSize(OFX_UI_FONT_SMALL, 6);
    gui->addLabelButton("RESET", false);
    gui->addToggle("DEBUG", &draw_debug);
    gui->addToggle("CAMERA", &use_camera);
    gui->addSlider("FACE_SEARCH_WINDOW", 0.05, 1.0, 0.2);
    gui->addRangeSlider("FACE_SIZE", 0.02, 1.0, 0.05, 0.4);
    gui->addRangeSlider("FLOW_THRESHOLD", 0.0, 3.0, 0.1, 0.5);
    gui->addIntSlider("FLOW_EROSION_SIZE", 1, 7, 5);
    gui->addLabelButton("1080x480", false);
    gui->addSlider("SQUID_SCALE", 0.5f, 2.0f, &squid.scale);
    gui->addSlider("JAGGY_SPACING", 1.0f, 100.0f, &jaggy_spacing);
    gui->addSlider("JAGGY_OFFSET", 0.5f, 30.0f, &jaggy_offset);
    gui->addSlider("FLUID_MOTION_SPEED", 1.0f, 50.0f, &fluid_motion_speed);
    gui->addSlider("FLUID_MOTION_RADIUS", 1.0f, 10.0f, &fluid_motion_radius);
//    gui->addMinimalSlider("SQUID_BODY_RADIUS", 10, 100, 40);
//    gui->addMinimalSlider("SQUID_BODY_DENSITY", 0.1, 1.0, 0.2);
//    gui->addMinimalSlider("SQUID_TENTACLE_DAMPING", 1.0, 20.0, 8.0);
    // Load settings
    gui->autoSizeToFitWidgets();
    ofAddListener(gui->newGUIEvent, this, &ofApp::guiEvent);
    gui->loadSettings("settings.xml");
}

void ofApp::setupPhysics()
{
    b2Vec2 gravity(0.0f, 1.0f);
    phys_world = ofPtr<b2World> ( new b2World(gravity ) );
    // Set up the world bounds
    b2BodyDef boundsBodyDef;
    boundsBodyDef.position.Set(0, 0);
    world_bounds = phys_world.get()->CreateBody(&boundsBodyDef);
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
    doCapture();

    if ((use_camera && camera.isFrameNew()) || (!use_camera && video.isFrameNew()) )
    {
        updateFrame();
        updateFlow();
        squid.updateObjectFinder(frame);
    }

    squid.update(delta_t, flow_high, frame, fluid);
    phys_world->Step(1.0f / kFrameRate, 6, 2);
    fluid.update();
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
        cv::resize(frame_full, frame_full, cv::Size(kCaptureWidth, kCaptureHeight));
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

    std::swap(flow_low_prev, flow_low);
    std::swap(flow_high_prev, flow_high);

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
    flow_behind = flow_high_prev & (255 - flow_high);
    flow_new = flow_high & ( 255 - flow_high_prev);
    contourfinder.findContours(flow_high);
}

//--------------------------------------------------------------
void ofApp::draw()
{
    ofSetColor(255, 255, 255, 255);
    ofxCv::drawMat(frame, 0, 0, ofGetWidth(), ofGetHeight());

//    drawParticles();
    if (draw_debug)
    {
        ofPushStyle();
        // Draw the optical flow maps
        ofSetColor(256, 0, 0, 196);
        ofEnableBlendMode(OF_BLENDMODE_ADD);
        ofxCv::drawMat(flow_high, 0, 0, ofGetWidth(), ofGetHeight());
        ofSetColor(224, 160, 58, 128);
        ofxCv::drawMat(flow_low, 0, 0, ofGetWidth(), ofGetHeight());
        ofDisableBlendMode();
        ofPopStyle();
    }

    ofEnableAlphaBlending();
    fluid.draw();
    ofDisableAlphaBlending();
    drawMotionEffects();
//    
//    ofSetLineWidth(3.0f);
//    ofSetColor(ofColor::pink);
//    ofPolyline test;
//    test.arc(ofGetWidth() / 2, ofGetHeight() / 2, 30.0f, 10.0f, 0.0f, 360.0f, 20);
//    for (int i = 0; i < test.size(); i ++){
//        ofPoint p = test[i];
//        ofPoint n = test.getNormalAtIndex(i);
//        ofLine(p, p + (30.0f * n));
//    }
//    test.draw();
    
    squid.draw(draw_debug);
    ofDrawBitmapStringHighlight(ofToString(ofGetFrameRate()) + "fps", kLabelOffset);
}

void ofApp::drawMotionEffects()
{
    // drawJaggies();
    drawFluid();
}

void ofApp::drawFluid(){
    float flow_scale = ofGetWidth() / (float)flow_high.cols;
    for (int i = 0; i < flow_low.rows; i += 5){
        for (int j = 0; j < flow_low.cols; j += 5){
            if (flow_high.at<uchar>(i,j)){
                ofPoint p = ofPoint(j, i);
                ofPoint motion_dir = toOf(flow.at<Point2f>(i, j));
                Point3_<uchar> frame_color = frame.at<Point3_<uchar> >(p.y * kFlowSubsample, p.x * kFlowSubsample);
                ofColor color = ofColor(frame_color.x, frame_color.y, frame_color.z);
                fluid.addTemporalForce(p * flow_scale, motion_dir * flow_scale, color, fluid_motion_radius, 0.5f, 0.5f );
            }
        }
    }
}

void ofApp::drawJaggies(){
    float flow_scale = ofGetWidth() / (float)flow_high.cols;

    for (int ci = 0; ci < contourfinder.size(); ci++)
    {
        ofPolyline contour = contourfinder.getPolyline(ci).getResampledBySpacing(jaggy_spacing);
        ofPolyline jaggies_a;
        ofPolyline jaggies_b;
        int label = contourfinder.getTracker().getLabelFromIndex(ci);
        cv::Point2f center = contourfinder.getCenter(ci);
        ofPoint motion_dir = toOf(flow.at<Point2f>(center.y, center.x));
        ofColor contour_color = getPersistentColor(label);

        ofSetLineWidth(2.0f);
        for (int ip = 0; ip < contour.size(); ip++){
            ofPoint p = contour[ip] * flow_scale;
            // For some reason the line winds such that the normals point inwards.
            ofPoint n = -(contour.getNormalAtIndex(ip));
            // Check if the normal is on the side of the motion
            if (motion_dir.dot(n) > 0.5){
                fluid.addTemporalForce(p, motion_dir * fluid_motion_speed, contour_color, fluid_motion_radius);
            }
            if (ip % 2 == 0){
                jaggies_a.addVertex(p + n * jaggy_offset);
                jaggies_b.addVertex(p);
            } else {
                jaggies_a.addVertex(p);
                jaggies_b.addVertex(p + n * jaggy_offset);
            }
        }
        jaggies_a.close();
        jaggies_b.close();

        ofSetLineWidth(10.0f);
        ofSetColor(ofColor::white);
        jaggies_a.draw();
        ofSetColor(contour_color);
        jaggies_b.draw();
    }
}

ofColor ofApp::getPersistentColor(int i){
    float h = fmod( (float) i * 17.0f, 255.0f );
    return ofColor::fromHsb(h, 200, 200);
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
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button)
{
}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h)
{
    reset();
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

    if (name == "RESET")
    {
        reset();
    }

    if (name == "FACE_SEARCH_WINDOW")
    {
        squid.face_search_window = ((ofxUISlider*) e.widget)->getScaledValue();
    }

    if (name == "FACE_SIZE")
    {
        squid.face_size_min = ((ofxUIRangeSlider*) e.widget)->getScaledValueLow();
        squid.face_size_max = ((ofxUIRangeSlider*) e.widget)->getScaledValueHigh();
    }

    if (name == "FLOW_THRESHOLD")
    {
        flow_threshold_low = ((ofxUIRangeSlider*) e.widget)->getScaledValueLow();
        flow_threshold_high = ((ofxUIRangeSlider*) e.widget)->getScaledValueHigh();
    }

    if (name == "FLOW_EROSION_SIZE")
    {
        flow_erosion_size = (int)(((ofxUIIntSlider*) e.widget)->getScaledValue());
    }

    if (name == "1080x480")
    {
        ofSetWindowShape(1080, 480);
        reset();
    }
}
