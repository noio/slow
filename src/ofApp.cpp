
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
    ofSetFrameRate(40);
    ofSetBackgroundAuto(false);
    ofSetMinMagFilters(GL_NEAREST, GL_NEAREST);
    ofClear(0, 0, 0, 255);
    ofEnableAlphaBlending();
    // Flow camera
    flowcam.setup(kCaptureWidth, kCaptureHeight, ofGetWidth(), ofGetHeight());
    // Set up Box2d
    setupPhysics();
    squid.setup(phys_world, &fluid, &flowcam);
    // Set up fluid
    fluid.allocate(ofGetWidth(), ofGetHeight(), kFluidScale);
    fluid.dissipation = 0.99;
    fluid.velocityDissipation = 0.99;
    fluid.setGravity(ofVec2f(0.0, 0.0));
    // Gui Setup
    setupGUI();
    need_setup = false;
}

void ofApp::setupGUI()
{
    // Set up control panel
    gui->removeWidgets();
    gui->setTheme(OFX_UI_THEME_HACKER);
    gui->setScrollAreaHeight(ofGetHeight());
    gui->setFontSize(OFX_UI_FONT_SMALL, 6);
    gui->addLabelButton("RESET", false);
    gui->addToggle("DEBUG", &draw_debug);
    gui->addToggle("CAMERA", false);
    gui->addSlider("FACE_SEARCH_WINDOW", 0.05, 1.0, 0.2);
    gui->addRangeSlider("FACE_SIZE", 0.02, 1.0, 0.05, 0.4);
    gui->addRangeSlider("FLOW_THRESHOLD", 0.0, 3.0, 0.1, 0.5);
    gui->addIntSlider("FLOW_EROSION_SIZE", 1, 7, 5);
    gui->addLabelButton("1080x480", false);
    gui->addSlider("SQUID_SCALE", 0.5f, 3.0f, 1.9f);
    gui->addSlider("JAGGY_SPACING", 1.0f, 100.0f, &jaggy_spacing);
    gui->addSlider("JAGGY_OFFSET", 0.5f, 30.0f, &jaggy_offset);
    gui->addSlider("FLUID_MOTION_SPEED", 1.0f, 50.0f, &fluid_motion_speed);
    gui->addSlider("FLUID_MOTION_RADIUS", 1.0f, 10.0f, &fluid_motion_radius);
    // Size
    ofAddListener(gui->newGUIEvent, this, &ofApp::guiEvent);
    // Position the GUI
    gui->setPosition(ofGetWidth() - gui->getSRect()->width, 0);
    gui->setHeight(ofGetHeight());
    gui->setScrollAreaHeight(ofGetHeight());
    gui->autoSizeToFitWidgets();
    // Load settings
    gui->loadSettings("settings.xml");
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
    flowcam.update(delta_t);
    squid.update(delta_t);
    phys_world->Step(1.0f / kFrameRate, 6, 2);
    fluid.update();
}


//--------------------------------------------------------------
void ofApp::draw()
{
    ofSetColor(255, 255, 255, 255);
    ofxCv::drawMat(flowcam.frame, 0, 0, ofGetWidth(), ofGetHeight());

    if (draw_debug) {
        ofPushStyle();
        // Draw the optical flow maps
        ofEnableBlendMode(OF_BLENDMODE_ADD);
        ofSetColor(224, 160, 58, 128);
        ofxCv::drawMat(flowcam.flow_low, 0, 0, ofGetWidth(), ofGetHeight());
        ofDisableBlendMode();
        ofPushMatrix();
        ofSetLineWidth(4.0);
        ofSetColor(255, 0, 0);
        ofScale(ofGetWidth() / (float)flowcam.flow_high.cols, ofGetHeight() / (float)flowcam.flow_high.rows);

        for (int i = 0; i < flowcam.contourfinder.size(); i ++) {
            flowcam.contourfinder.getPolyline(i).draw();
        }

        ofPopMatrix();
        ofPopStyle();
    }

    ofEnableAlphaBlending();
    fluid.draw();
    ofDisableAlphaBlending();
    drawMotionEffects();
    squid.draw(draw_debug);
    ofDrawBitmapStringHighlight(ofToString(ofGetFrameRate()) + "fps", kLabelOffset);
}

void ofApp::drawMotionEffects()
{
    // drawJaggies();
    drawFluid();
}

void ofApp::drawFluid()
{
    float flow_scale = ofGetWidth() / (float)flowcam.flow_high.cols;

    for (int i = 0; i < flowcam.flow_low.rows; i += 5) {
        for (int j = 0; j < flowcam.flow_low.cols; j += 5) {
            if (flowcam.flow_high.at<uchar>(i, j)) {
                ofPoint p = ofPoint(j, i);
                ofPoint motion_dir = toOf(flowcam.flow.at<Point2f>(i, j));
                Point3_<uchar> frame_color = flowcam.frame.at<Point3_<uchar> >(p.y * kFlowSubsample, p.x * kFlowSubsample);
                ofColor color = ofColor(frame_color.x, frame_color.y, frame_color.z);
                fluid.addTemporalForce(p * flow_scale, motion_dir * flow_scale, color, fluid_motion_radius, 0.5f, 0.5f );
            }
        }
    }
}

void ofApp::drawJaggies()
{
    float flow_scale = ofGetWidth() / (float)flowcam.flow_high.cols;

    for (int ci = 0; ci < flowcam.contourfinder.size(); ci++) {
        ofPolyline contour = flowcam.contourfinder.getPolyline(ci).getResampledBySpacing(jaggy_spacing);
        ofPolyline jaggies_a;
        ofPolyline jaggies_b;
        int label = flowcam.contourfinder.getTracker().getLabelFromIndex(ci);
        cv::Point2f center = flowcam.contourfinder.getCenter(ci);
        ofPoint motion_dir = toOf(flowcam.flow.at<Point2f>(center.y, center.x));
        ofColor contour_color = getPersistentColor(label);
        ofSetLineWidth(2.0f);

        for (int ip = 0; ip < contour.size(); ip++) {
            ofPoint p = contour[ip] * flow_scale;
            // For some reason the line winds such that the normals point inwards.
            ofPoint n = -(contour.getNormalAtIndex(ip));

            // Check if the normal is on the side of the motion
            if (motion_dir.dot(n) > 0.5) {
                fluid.addTemporalForce(p, motion_dir * fluid_motion_speed, contour_color, fluid_motion_radius);
            }

            if (ip % 2 == 0) {
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

ofColor ofApp::getPersistentColor(int i)
{
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
    switch (key) {
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
        need_setup = true;
    }

    if (name == "CAMERA") {
        flowcam.setUseCamera( ((ofxUIToggle*) e.widget)->getValue() );
    }

    if (name == "FACE_SEARCH_WINDOW") {
        squid.face_search_window = ((ofxUISlider*) e.widget)->getScaledValue();
    }

    if (name == "FACE_SIZE") {
        squid.face_size_min = ((ofxUIRangeSlider*) e.widget)->getScaledValueLow();
        squid.face_size_max = ((ofxUIRangeSlider*) e.widget)->getScaledValueHigh();
    }

    if (name == "FLOW_THRESHOLD") {
        flowcam.flow_threshold_low = ((ofxUIRangeSlider*) e.widget)->getScaledValueLow();
        flowcam.flow_threshold_high = ((ofxUIRangeSlider*) e.widget)->getScaledValueHigh();
    }

    if (name == "FLOW_EROSION_SIZE") {
        flowcam.setFlowErosionSize((int)(((ofxUIIntSlider*) e.widget)->getScaledValue()));
    }

    if (name == "SQUID_SCALE") {
        squid.setScale(((ofxUISlider*) e.widget)->getScaledValue());
    }

    if (name == "1080x480") {
        ofSetWindowShape(1080, 480);
        need_setup = true;
    }
}
