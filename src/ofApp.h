#pragma once

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxUI.h"
#include "Fluid.h"


class ofApp : public ofBaseApp{

    
	public:
        ofApp() : gui("EDIT")
        { };
    
		void setup();
		void update();
		void draw();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
        void guiEvent(ofxUIEventArgs &e);
    
        ofxUISuperCanvas gui;
    
        cv::Mat erode_kernel, frame, frame_gray, magnitude, angle, flow_low, flow_low_prev, flow_high, sensitivity;
    
        cv::Rect roi;
    
        ofVideoPlayer video;
        ofVideoGrabber camera;
        ofxCv::FlowFarneback opticalflow;
		
        ofVec2f pMouse;
    
        FluidSolver fluid;
        ofTexture fluid_texture;
};
