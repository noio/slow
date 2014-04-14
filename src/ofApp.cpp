
#include "ofApp.h"
#include "constants.h"

using namespace ofxCv;

using std::cout;
using std::endl;

//--------------------------------------------------------------
void ofApp::setup(){
    // GENERIC OPENFRAMEWORKS SETUP
    ofSetFrameRate(40);
    ofBackground(0, 0, 0);
    
    // SET UP CAMERA/VIDEO AND RESOLUTION
    camera.initGrabber(kCaptureWidth, kCaptureHeight);
    video.loadMovie("videos/damrak/damrak_3.mov");
    video.play();
    
    float ratio = (float)kScreenWidth / (float)kScreenHeight;
    int w = std::min(kCaptureWidth, static_cast<int>(kCaptureHeight * ratio));
    int h = std::min(kCaptureHeight, static_cast<int>(kCaptureWidth / ratio));
    roi = cv::Rect((kCaptureWidth - w) / 2, (kCaptureHeight - h) / 2, w, h);
    
    
    // SET UP OPTICAL FLOW
    open_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                         cv::Size(2 * kFlowErosionSize + 1, 2 * kFlowErosionSize + 1),
                         cv::Point(kFlowErosionSize, kFlowErosionSize));

    opticalflow.setPyramidScale(0.5);
    opticalflow.setNumLevels(2);
    opticalflow.setWindowSize(7);
    opticalflow.setNumIterations(3);
    opticalflow.setPolyN(5);
    opticalflow.setPolySigma(1.2);
    
    // SET UP FLUID DYNAMICS
    fluid.setup(kFluidWidth, kFluidHeight);
    
    // SET UP PARTICLE SYSTEM
    particles.setup(kGameWidth, kGameHeight, 3);
    particles.setTimeStep(1/30.0);
    
    // SET UP CONTROL PANEL
    gui.addSpacer();
    gui.addSlider("VISCOSITY", 0, 0.001, 0.0008)->setLabelPrecision(4);
    gui.addSlider("DECAY", 0.8, 0.999, 0.950)->setLabelPrecision(4);
    gui.addSlider("DIFFUSION", 0.00001, 0.0001, 0.0001)->setLabelPrecision(5);
    gui.autoSizeToFitWidgets();
    gui.setPosition(kScreenWidth-212, 0);
    
    ofAddListener(gui.newGUIEvent,this,&ofApp::guiEvent);
}

//--------------------------------------------------------------
void ofApp::update(){
    
    camera.update();
    video.update();

//    cv::Mat frame_full = toCv(camera);
    cv::Mat frame_full = toCv(video.getPixelsRef());

    cv::flip(frame_full(roi), frame, 1);
    
    fluid.update();
    
    // COMPUTING OPTICAL FLOW
    if (video.isFrameNew()){
        updateFlow();
    }
    
    
    // AFFECT FLUID
    for (int x = 0; x < flow_high.cols; x++){
        for (int y = 0; y < flow_high.rows; y++){
            Vec2f f = flow.at<Vec2f>(y, x);
            float x_ = (float) x / flow_high.cols;
            float y_ = (float) y / flow_high.rows;

            if (flow_behind.at<bool>(y,x)){
                
                if ((particles.size() < 1000) && ofRandom(1.0) < 0.1) {
                    Particle p(x_ * kScreenWidth, y_ * kScreenHeight, ofRandom(-.5,.5), ofRandom(-.5, .5));
                    particles.add(p);
                }

                fluid.add_velocity(x_, y_, -MAX(-20, MIN(.5 * f[0], 20)), -MAX(-20, MIN(.5 * f[1], 20)));
                fluid.add_density(x_, y_, 0.03 * (random() % 10));
            }
            if (flow_new.at<bool>(y,x)){
                fluid.add_velocity(x_, y_, MAX(-20, MIN(.2 * f[0], 20)), MAX(-20, MIN(.2 * f[1], 20)));
            }
        }
    }
    
    updateParticles();
}

void ofApp::updateFlow(){
    cv::cvtColor(frame, frame_gray, CV_BGR2GRAY);
    cv::pyrDown(frame_gray, frame_gray);
    cv::pyrDown(frame_gray, frame_gray);
    opticalflow.calcOpticalFlow(frame_gray);
    
    flow = opticalflow.getFlow();
    
    // ofxCV wrapper returns a 1x1 flow image after the first optical flow computation.
    if (flow.cols == 1){
        flow_low_prev = cv::Mat::zeros(frame_gray.rows, frame_gray.cols, CV_8U);
        flow_high_prev = cv::Mat::zeros(frame_gray.rows, frame_gray.cols, CV_8U);
        flow = cv::Mat::zeros(frame_gray.rows, frame_gray.cols, CV_32FC2);
    }
    
    std::vector<cv::Mat> xy(2);
    cv::split(flow, xy);
    cv::cartToPolar(xy[0], xy[1], magnitude, angle, true);
    
    // Compute the low speed mask
    cv::threshold(magnitude, magnitude, kFlowLowThreshold, 1, cv::THRESH_TOZERO);
    flow_low = magnitude > 0;
    cv::erode(flow_low, flow_low, open_kernel);
    cv::dilate(flow_low, flow_low, open_kernel);
    
    // Compute the high speed mask
//    cv::add(sensitivity, flow_low, sensitivity, cv::noArray(), CV_32F);
//    sensitivity *= kFlowSensitivityDecay;
//    flow_high = magnitude >  sensitivity * kFlowSensitivityMultiplier;
    
    cv::threshold(magnitude, flow_high, kFlowHighThreshold, 1, cv::THRESH_TOZERO);
    flow_high = flow_high > 0; // & flow_low_prev > 0;

    cv::erode(flow_high, flow_high, open_kernel);
//    cv::dilate(flow_high, flow_high, open_kernel_small);
    
    flow_behind = flow_high_prev & (255 - flow_high);
    flow_new = flow_high & ( 255 - flow_high_prev);
    
    std::swap(flow_low_prev, flow_low);
    std::swap(flow_high_prev, flow_high);
    
}

void ofApp::updateParticles(){
    particles.setupForces();
    
    for(int i = 0; i < particles.size(); i++) {
        Particle& cur = particles[i];
        // global force on other particles
        particles.addRepulsionForce(cur, 50, 5.0);
        Velocity v = fluid.velocity_at(cur.x / kGameWidth, cur.y / kGameHeight);
        cur.xf += v.u * kGameWidth / 30.0;
        cur.yf += v.v * kGameHeight / 30.0;
        // forces on this particle
        cur.bounceOffWalls(0, 0, kGameWidth, kGameHeight);
        cur.addDampingForce();
    }
    
    particles.update();
}



//--------------------------------------------------------------
void ofApp::draw(){
    ofSetColor(255,255,255,255);

    ofxCv::drawMat(frame, 0, 0, kScreenWidth, kScreenHeight);

    ofxCv::drawMat(flow_behind, 0, 0);
    ofxCv::drawMat(flow_new, 0, flow.rows);
    
    opticalflow.draw(0,0, kScreenWidth, kScreenHeight);

    unsigned char pixels[kFluidWidth*kFluidHeight];
    fluid.fill_texture(pixels);
    fluid_texture.loadData(pixels, kFluidWidth, kFluidHeight, GL_ALPHA);
    fluid_texture.draw(0,0,kScreenWidth,kScreenHeight);
    
    triangulator.reset();

    for(int i = 0; i < particles.size(); i++) {
        Particle& cur = particles[i];
        triangulator.addPoint(cur.x, cur.y, 0);
    }
    
    triangulator.triangulate();
    
    vector<ofMeshFace> tris = triangulator.triangleMesh.getUniqueFaces();
    for(std::vector<ofMeshFace>::iterator it = tris.begin(); it != tris.end(); ++it){
        ofPoint p0 = it->getVertex(0);
        ofPoint p1 = it->getVertex(1);
        ofPoint p2 = it->getVertex(2);
        float d = p0.distance(p1) + p1.distance(p2) + p2.distance(p0);
        if (d < 1000){
            ofSetColor(0, 0, 0, 10000 / d);
            ofTriangle(p0,p1,p2);
        }
    }
    
    particles.draw();
//
    
//    ofSetColor(0, 0, 0);
//    triangulator.triangleMesh.drawFaces();
    
//    triangulator.triangleMesh;
    
    ofDrawBitmapString(ofToString(ofGetFrameRate())+"fps", 10, 15);
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    if (key == ' '){
        fluid.update();
    }
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){
    
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}

void ofApp::guiEvent(ofxUIEventArgs &e){
    string name = e.widget->getName();
	int kind = e.widget->getKind();
    
    if(name == "VISCOSITY")
	{
		ofxUISlider *slider = (ofxUISlider *) e.widget;
		fluid.viscosity = slider->getScaledValue();
	}
    if(name == "DECAY")
	{
		ofxUISlider *slider = (ofxUISlider *) e.widget;
		fluid.density_decay = slider->getScaledValue();
	}
    if(name == "DIFFUSION"){
        ofxUISlider *slider = (ofxUISlider *) e.widget;
        fluid.diffusion = slider->getScaledValue();
    }
}
