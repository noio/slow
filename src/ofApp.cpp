
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
    // Set up camera and video
    camera.initGrabber(kCaptureWidth, kCaptureHeight);
    video.loadMovie("videos/damrak/damrak_3.mov");
    video.setVolume(0);
    video.play();
    //
    // Set up optical flow
    open_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                            cv::Size(2 * kFlowErosionSize + 1, 2 * kFlowErosionSize + 1),
                                            cv::Point(kFlowErosionSize, kFlowErosionSize));
    opticalflow.setPyramidScale(0.5);
    opticalflow.setNumLevels(2);
    opticalflow.setWindowSize(7);
    opticalflow.setNumIterations(3);
    opticalflow.setPolyN(5);
    opticalflow.setPolySigma(1.2);
    //
    // Set up fluid dynamics
    fluid.setup(kFluidWidth, kFluidHeight);
    //
    // Set up the particle system
    particles.setup(kGameWidth, kGameHeight, 3, kMaxParticles);
    particles.setTimeStep(1 / kFrameRate);
    //
    // Set up Box2d
    b2Vec2 gravity(0.0f, 0.0f);
    phys_world = ofPtr<b2World> ( new b2World(gravity ) );
    // Set up the world bounds
    b2BodyDef boundsBodyDef;
	boundsBodyDef.position.Set(0, 0);
	b2Body* bounds = phys_world.get()->CreateBody(&boundsBodyDef);
	b2EdgeShape shape;
    b2AABB rec = ofToB2(ofRectangle(-kGameSizePadding, -kGameSizePadding,
                                    (kGameWidth + kGameSizePadding * 2), (kGameHeight + kGameSizePadding * 2)));
    //right wall
	shape.Set(b2Vec2(rec.upperBound.x, rec.lowerBound.y), b2Vec2(rec.upperBound.x, rec.upperBound.y));
	bounds->CreateFixture(&shape, 0.0f);
	//left wall
	shape.Set(b2Vec2(rec.lowerBound.x, rec.lowerBound.y), b2Vec2(rec.lowerBound.x, rec.upperBound.y));
	bounds->CreateFixture(&shape, 0.0f);
	// top wall
	shape.Set(b2Vec2(rec.lowerBound.x, rec.lowerBound.y), b2Vec2(rec.upperBound.x, rec.lowerBound.y));
	bounds->CreateFixture(&shape, 0.0f);
	// bottom wall
	shape.Set(b2Vec2(rec.lowerBound.x, rec.upperBound.y), b2Vec2(rec.upperBound.x, rec.upperBound.y));
	bounds->CreateFixture(&shape, 0.0f);
    
    squid.setup(phys_world);
    //
    // Set up control panel
    gui.addSpacer();
    gui.addSlider("VISCOSITY", 0, 0.001, 0.0008)->setLabelPrecision(4);
    gui.addSlider("DECAY", 0.8, 0.999, 0.950)->setLabelPrecision(4);
    gui.addSlider("DIFFUSION", 0.00001, 0.0001, 0.0001)->setLabelPrecision(5);
    gui.addToggle("DEBUG", false);
    gui.autoSizeToFitWidgets();
    gui.setPosition(kScreenWidth - 212, 0);
    ofAddListener(gui.newGUIEvent, this, &ofApp::guiEvent);
    gui.loadSettings("settings.xml");
}

//--------------------------------------------------------------
void ofApp::update()
{
    delta_t = ofGetLastFrameTime();
    if (draw_debug)
    {
        ofClear(0, 0, 0, 255);
    }
    particles.setupForces();
    camera.update();
    video.update();
//    cv::Mat frame_full = toCv(camera);
    cv::Mat frame_full = toCv(video.getPixelsRef());
    cv::flip(frame_full(kCaptureROI), frame, 1);
    // Compute optical flow
    if (video.isFrameNew())
    {
        updateFlow();
        updateMotionEffect();
    }
    squid.update(delta_t, flow_high, draw_debug);
    // Update physics
    phys_world->Step(1.0f / kFrameRate, 6, 2);
    updateFluid();
    updateParticles();
}



void ofApp::updateFlow()
{
    cv::cvtColor(frame, frame_gray, CV_BGR2GRAY);
    cv::pyrDown(frame_gray, frame_gray);
    cv::pyrDown(frame_gray, frame_gray);
    assert(frame_gray.cols == kFlowWidth && frame_gray.rows == kFlowHeight);
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
    cv::threshold(magnitude, magnitude, kFlowLowThreshold, 1, cv::THRESH_TOZERO);
    flow_low = magnitude > 0;
    cv::erode(flow_low, flow_low, open_kernel);
    cv::dilate(flow_low, flow_low, open_kernel);
    //
    // Compute the high speed mask
//    cv::add(sensitivity, flow_low, sensitivity, cv::noArray(), CV_32F);
//    sensitivity *= kFlowSensitivityDecay;
//    flow_high = magnitude >  sensitivity * kFlowSensitivityMultiplier;
    cv::threshold(magnitude, flow_high, kFlowHighThreshold, 1, cv::THRESH_TOZERO);
    flow_high = flow_high > 0; // & flow_low_prev > 0;
    cv::erode(flow_high, flow_high, open_kernel);
//    cv::dilate(flow_high, flow_high, open_kernel_small);
    contourfinder.findContours(flow_high);
    flow_behind = flow_high_prev & (255 - flow_high);
    flow_new = flow_high & ( 255 - flow_high_prev);
    std::swap(flow_low_prev, flow_low);
    std::swap(flow_high_prev, flow_high);
}



void ofApp::updateMotionEffect()
{
    const vector<ofPolyline>& polylines = contourfinder.getPolylines();
    for (int i = 0; i < polylines.size(); i++)
    {
        const ofPolyline& cur = polylines[i];
        ofPoint motion = opticalflow.getAverageFlowInRegion(cur.getBoundingBox());
        ofPoint centroid = cur.getCentroid2D() / kFlowSize;
        ofPoint force = motion * -50;
        //        centroid.x += ofRandom(-10, 10);
        //        centroid.y += ofRandom(-10, 10);
        fluid.add_velocity(centroid.x, centroid.y, force.x, force.y);
        fluid.add_density(centroid.x, centroid.y, 1);
        centroid = centroid * kGameSize;
        particles.addRepulsionForce(centroid.x, centroid.y, 100, 100);
        ofPolyline resampled = cur.getResampledBySpacing(4.0);
        const vector<ofPoint>& vertices = resampled.getVertices();
        for (int j = 0; j < vertices.size(); j++)
        {
            ofPoint pos = vertices[j] / kFlowSize;
            ofPoint normal = resampled.getNormalAtIndex(j);
            if (normal.angle(motion) < 45)
            {
                pos = pos * kGameSize;
                Particle p(pos.x + ofRandom(-1, 1), pos.y + ofRandom(-1, 1), force.x, force.y);
                p.color = ofColor::fromHsb(fmod(ofGetElapsedTimef() * 16 + motion.angle(ofPoint(1, 0, 0)), 255), 100, ofRandom(100));
                particles.add(p);
            }
            else
            {
                pos = pos / kFlowSize;
                fluid.add_velocity(pos.x, pos.y, 20 * normal.x, 20 * normal.y);
                //                ofSetColor(255,255,255,255);
                //                ofLine(pos.x * kScreenWidth, pos.y * kScreenHeight, pos.x * kScreenWidth + 10 * normal.x, pos.y * kScreenHeight + 10 * normal.y);
            }
        }
    }
}

void ofApp::updateFluid()
{
    for (int x = 0; x < flow_high.cols; x++)
    {
        for (int y = 0; y < flow_high.rows; y++)
        {
            Vec2f f = flow.at<Vec2f>(y, x);
            float x_ = (float) x / flow_high.cols;
            float y_ = (float) y / flow_high.rows;
//            if (flow_behind.at<bool>(y,x)){
//
//                if (ofRandom(1.0) < 0.05) {
//                    Particle p(x_ * kScreenWidth, y_ * kScreenHeight, 0,0);
//                    p.color = ofColor::fromHsb(fmod(ofGetElapsedTimef() * 16 + x_ * 64, 255), 100, 100);
//                    particles.add(p);
//                }
//
//                fluid.add_velocity(x_, y_, -MAX(-20, MIN(.5 * f[0], 20)), -MAX(-20, MIN(.5 * f[1], 20)));
//                fluid.add_density(x_, y_, 0.03 * (random() % 10));
//            }
//            if (flow_new.at<bool>(y,x)){
////                fluid.add_velocity(x_, y_, MAX(-20, MIN(.5 * f[0], 20)), MAX(-20, MIN(.5 * f[1], 20)));
//            }
        }
    }
    fluid.update();
}

void ofApp::updateParticles()
{
    particles.clean();
    for(int i = 0; i < particles.size(); i++)
    {
        Particle& cur = particles[i];
        // global force on other particles
        particles.addRepulsionForce(cur, 20, 1.0);
//        particles.addAttractionForce(cur, 100, 0.1);
        Velocity v = fluid.velocity_at(cur.x / kGameWidth, cur.y / kGameHeight);
//        cur.xf += v.u * kGameWidth / 30.0;
//        cur.yf += v.v * kGameHeight / 30.0;
        // forces on this particle
        cur.bounceOffWalls(0, 0, kGameWidth, kGameHeight);
        cur.addDampingForce(0.05);
    }
    particles.update();
}



//--------------------------------------------------------------
void ofApp::draw()
{
    ofSetColor(255, 255, 255, 255);
    if (!draw_debug)
    {
        ofxCv::drawMat(frame, 0, 0, kScreenWidth, kScreenHeight);
        drawParticles();
    }
    if (draw_debug)
    {
        ofSetColor(255, 0, 0, 100);
        ofxCv::drawMat(flow_high, 0, 0, kScreenWidth, kScreenHeight);
    }
//    unsigned char pixels[kFluidWidth*kFluidHeight];
//    fluid.fill_texture(pixels);
//    fluid_texture.loadData(pixels, kFluidWidth, kFluidHeight, GL_ALPHA);
//    fluid_texture.draw(0,0,kScreenWidth,kScreenHeight);
    squid.draw();
    ofSetColor(255, 0, 255);
    ofDrawBitmapString(ofToString(ofGetFrameRate()) + "fps", 10, 15);
}

void ofApp::drawParticles()
{
    triangulator.reset();
    for(int i = 0; i < particles.size(); i++)
    {
        Particle& cur = particles[i];
        if (cur.alive)
        {
            ofCircle(cur.x, cur.y, 2);
            cur.color.a = cur.life * 4;
            triangulator.addPoint(cur.x, cur.y, 0, cur.color);
        }
    }
    triangulator.triangulate();
//    ofNoFill();

    vector<ofMeshFace> tris = triangulator.triangleMesh.getUniqueFaces();
    for(std::vector<ofMeshFace>::iterator it = tris.begin(); it != tris.end(); ++it)
    {
        ofPoint p0 = it->getVertex(0);
        ofPoint p1 = it->getVertex(1);
        ofPoint p2 = it->getVertex(2);
        float d = p0.distance(p1) + p1.distance(p2) + p2.distance(p0);
        if (d < 256)
        {
            ofColor c0 = it->getColor(0), c1 = it->getColor(1), c2 = it->getColor(2);
            float alpha = (c0.a + c1.a + c2.a) * 128 / d;
            ofColor c = it->getColor(0) + it->getColor(1) + it->getColor(2);
            ofSetColor(c, alpha);
            ofTriangle(p0, p1, p2);
        }
    }
}


void ofApp::exit()
{
    gui.saveSettings("settings.xml");
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key)
{
    if (key == ' ')
    {
        fluid.update();
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
    if(name == "VISCOSITY")
    {
        ofxUISlider* slider = (ofxUISlider*) e.widget;
        fluid.viscosity = slider->getScaledValue();
    }
    if(name == "DECAY")
    {
        ofxUISlider* slider = (ofxUISlider*) e.widget;
        fluid.density_decay = slider->getScaledValue();
    }
    if(name == "DIFFUSION")
    {
        ofxUISlider* slider = (ofxUISlider*) e.widget;
        fluid.diffusion = slider->getScaledValue();
    }
    if (name == "DEBUG")
    {
        ofxUIToggle* toggle = (ofxUIToggle*) e.widget;
        draw_debug = toggle->getValue();
    }
}
