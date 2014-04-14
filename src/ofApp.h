#pragma once

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxUI.h"
#include "ofxDelaunay.h"

#include "Fluid.h"
#include "Particle.h"


class ofApp : public ofBaseApp{

    
	public:
        ofApp() : gui("EDIT")
        { };
    
		void setup();
		void update();
		void draw();
    
        void updateFlow();
        void updateParticles();

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
    
        cv::Mat open_kernel;
        cv::Mat frame, frame_gray;
        cv::Mat magnitude, angle, flow, flow_low, flow_low_prev, flow_high, flow_high_prev, flow_behind, flow_new;
    
        cv::Rect roi;
    
        ofVideoPlayer video;
        ofVideoGrabber camera;
        ofxCv::FlowFarneback opticalflow;
		
        ofVec2f pMouse;
    
        FluidSolver fluid;
        ofTexture fluid_texture;
    
        std::vector<Particle> particles;
    
        ofxDelaunay triangulator;
};
