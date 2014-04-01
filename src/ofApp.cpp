
#include "ofApp.h"
#include "constants.h"

using namespace ofxCv;

using std::cout;
using std::endl;

//--------------------------------------------------------------
void ofApp::setup(){
    ofSetFrameRate(30);
    ofBackground(0, 0, 0);
    
    camera.setDeviceID(1);
    camera.initGrabber(kCaptureWidth, kCaptureHeight);
    
    float ratio = (float)kScreenWidth / (float)kScreenHeight;
    int w = std::min(kCaptureWidth, static_cast<int>(kCaptureHeight * ratio));
    int h = std::min(kCaptureHeight, static_cast<int>(kCaptureWidth / ratio));
    roi = cv::Rect((kCaptureWidth - w) / 2, (kCaptureHeight - h) / 2, w, h);
    
    fluid.setup(kFluidWidth, kFluidHeight);
    fluid.enableRGB(true).setFadeSpeed(0.002).setDeltaT(0.5).setVisc(0.00002).setColorDiffusion(0);

    fluid_drawer.setup(&fluid);
//    fluid_drawer.setDrawMode(msa::fluid::kDrawVectors);
    
    pMouse = msa::getWindowCenter();
}

//--------------------------------------------------------------
void ofApp::update(){
    fluid.update();
    
    camera.update();
    
    cv::Mat frame_full = toCv(camera);
    frame = frame_full(roi);
    
    cv::cvtColor(frame, frame_gray, CV_BGR2GRAY);
    cv::pyrDown(frame_gray, frame_gray);
//    cv::pyrDown(frame_gray, frame_gray);
//    opticalflow.calcOpticalFlow(frame_gray);
    
    
    
}

// add force and dye to fluid, and create particles
void ofApp::addToFluid(ofVec2f pos, ofVec2f vel, bool addColor, bool addForce) {
    float speed = vel.x * vel.x  + vel.y * vel.y * msa::getWindowAspectRatio() * msa::getWindowAspectRatio();    // balance the x and y components of speed with the screen aspect ratio
    if(speed > 0) {
		pos.x = ofClamp(pos.x, 0.0f, 1.0f);
		pos.y = ofClamp(pos.y, 0.0f, 1.0f);
		
        int index = fluid.getIndexForPos(pos);
		
		if(addColor) {
            //			Color drawColor(CM_HSV, (getElapsedFrames() % 360) / 360.0f, 1, 1);
			
			fluid.addColorAtIndex(index, 100);
			
//			if(drawParticles)
//				particleSystem.addParticles(pos * ofVec2f(ofGetWindowSize()), 10);
		}
		
		if(addForce)
			fluid.addForceAtIndex(index, vel * 300);
		
    }
}


//--------------------------------------------------------------
void ofApp::draw(){
    ofxCv::drawMat(frame, 0, 0, kScreenWidth, kScreenHeight);
//    camera.draw(0, 0);
    cv::Mat flow = opticalflow.getFlow();
    ofxCv::drawMat(flow, 0, 0);
//    opticalflow.draw(0,0, kScreenWidth, kScreenHeight);
    ofDrawBitmapString(ofToString(ofGetFrameRate())+"fps", 10, 15);
    
    fluid_drawer.draw(0, 0, kScreenWidth, kScreenHeight);
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){
    ofVec2f eventPos = ofVec2f(x, y);
	ofVec2f mouseNorm = ofVec2f(eventPos) / ofGetWindowSize();
	ofVec2f mouseVel = ofVec2f(eventPos - pMouse) / ofGetWindowSize();
	addToFluid(mouseNorm, mouseVel, true, true);
	pMouse = eventPos;
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
