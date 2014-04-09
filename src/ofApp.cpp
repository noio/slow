
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
    video.loadMovie("test.mp4");
    video.play();
    
    float ratio = (float)kScreenWidth / (float)kScreenHeight;
    int w = std::min(kCaptureWidth, static_cast<int>(kCaptureHeight * ratio));
    int h = std::min(kCaptureHeight, static_cast<int>(kCaptureWidth / ratio));
    roi = cv::Rect((kCaptureWidth - w) / 2, (kCaptureHeight - h) / 2, w, h);
    
    
    // SET UP OPTICAL FLOW
    cv::getStructuringElement(cv::MORPH_ELLIPSE,
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
    
    // SET UP CONTROL PANEL
    gui.addSpacer();
    gui.addSlider("VISCOSITY", 0, 0.001, 0.0008)->setLabelPrecision(4);
    gui.addSlider("DECAY", 0.8, 0.999, 0.96)->setLabelPrecision(4);
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
    updateFlow();
    
    // AFFECT FLUID
    for (int x = 0; x < flow_high.cols; x++){
        for (int y = 0; y < flow_high.rows; y++){
            if (flow_high.at<bool>(y,x)){
                Vec2f f = flow.at<Vec2f>(y, x);
                float x_ = (float) x / flow_high.cols;
                float y_ = (float) y / flow_high.rows;
                fluid.add_velocity(x_, y_, -MAX(-20, MIN(2 * f[0], 20)), -MAX(-20, MIN(2 * f[1], 20)));
                fluid.add_density(x_, y_, 1);
            }
        }
    }
}

void ofApp::updateFlow(){
    cv::cvtColor(frame, frame_gray, CV_BGR2GRAY);
    cv::pyrDown(frame_gray, frame_gray);
    cv::pyrDown(frame_gray, frame_gray);
    opticalflow.calcOpticalFlow(frame_gray);
    
    flow = opticalflow.getFlow();
    
    // ofxCV wrapper returns a 1x1 flow image after the first optical flow computation.
    if (flow.cols == 1){
        sensitivity = cv::Mat::zeros(frame_gray.rows, frame_gray.cols, CV_32F);
        flow_low_prev = cv::Mat::zeros(frame_gray.rows, frame_gray.cols, CV_32F);
        flow = cv::Mat::zeros(frame_gray.rows, frame_gray.cols, CV_32FC2);
    }
    
    std::vector<cv::Mat> xy(2);
    cv::split(flow, xy);
    cv::cartToPolar(xy[0], xy[1], magnitude, angle, true);
    
    // Compute the low speed mask
    cv::threshold(magnitude, magnitude, kFlowLowThreshold, 1, cv::THRESH_TOZERO);
    flow_low = magnitude > 0;
    cv::erode(flow_low, flow_low, erode_kernel);
    cv::dilate(flow_low, flow_low, erode_kernel);
    
    // Compute the high speed mask
//    cv::add(sensitivity, flow_low, sensitivity, cv::noArray(), CV_32F);
//    sensitivity *= kFlowSensitivityDecay;
//    flow_high = magnitude >  sensitivity * kFlowSensitivityMultiplier;
    
    cv::threshold(magnitude, flow_high, kFlowHighThreshold, 1, cv::THRESH_TOZERO);
    flow_high = flow_high > 0; // & flow_low_prev > 0;

    
    cv::erode(flow_high, flow_high, erode_kernel);
    cv::dilate(flow_high, flow_high, erode_kernel);
    
    std::swap(flow_low_prev, flow_low);
    
}



//--------------------------------------------------------------
void ofApp::draw(){
    ofxCv::drawMat(frame, 0, 0, kScreenWidth, kScreenHeight);
    
    ofxCv::drawMat(flow_high, 0, 0);
//    opticalflow.draw(0,0, kScreenWidth, kScreenHeight);

    unsigned char pixels[kFluidWidth*kFluidHeight];
    fluid.fill_texture(pixels);
    fluid_texture.loadData(pixels, kFluidWidth, kFluidHeight, GL_ALPHA);
    fluid_texture.draw(0,0,kScreenWidth,kScreenHeight);
    
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
}
