
#include "ofApp.h"
#include "constants.h"

using namespace ofxCv;

using std::cout;
using std::endl;

//--------------------------------------------------------------
void ofApp::setup(){
    ofSetFrameRate(120);
    ofBackground(0, 0, 0);
    
//    camera.setDeviceID(0);
    camera.initGrabber(kCaptureWidth, kCaptureHeight);
    
    float ratio = (float)kScreenWidth / (float)kScreenHeight;
    int w = std::min(kCaptureWidth, static_cast<int>(kCaptureHeight * ratio));
    int h = std::min(kCaptureHeight, static_cast<int>(kCaptureWidth / ratio));
    roi = cv::Rect((kCaptureWidth - w) / 2, (kCaptureHeight - h) / 2, w, h);
    
    fluid.setup(kFluidWidth, kFluidHeight);
}

//--------------------------------------------------------------
void ofApp::update(){
    
//    camera.update();
//    cv::Mat frame_full = toCv(camera);
//    frame = frame_full(roi);
    
    fluid.update();
    
//    cv::cvtColor(frame, frame_gray, CV_BGR2GRAY);
//    cv::pyrDown(frame_gray, frame_gray);
//    cv::pyrDown(frame_gray, frame_gray);
//    opticalflow.calcOpticalFlow(frame_gray);

}



//--------------------------------------------------------------
void ofApp::draw(){
//    ofxCv::drawMat(frame, 0, 0, kScreenWidth, kScreenHeight);
//    camera.draw(0, 0);
//    cv::Mat flow = opticalflow.getFlow();
//    ofxCv::drawMat(flow, 0, 0);
//    opticalflow.draw(0,0, kScreenWidth, kScreenHeight);

    
    unsigned char pixels[kFluidWidth*kFluidHeight];
    fluid.fill_texture(pixels);
    fluid_texture.loadData(pixels, kFluidWidth, kFluidHeight, GL_LUMINANCE);
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
