#pragma once

#include "ofMain.h"
#include "ofxCv.h"
#include "MSAFluid.h"

class ofApp : public ofBaseApp{
    
    

	public:
    
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
    
        void addToFluid(ofVec2f pos, ofVec2f vel, bool addColor, bool addForce);
    
        cv::Mat frame, frame_gray;
    
        cv::Rect roi;
    
        ofVideoGrabber camera;
        ofxCv::FlowFarneback opticalflow;
		
        ofVec2f pMouse;
    
        msa::fluid::Solver fluid;
        msa::fluid::DrawerGl fluid_drawer;
};
