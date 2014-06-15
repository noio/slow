
#ifndef SLOW_FLOWCAM_H_
#define SLOW_FLOWCAM_H_

#include "ofMain.h"
#include "ofxCv.h"

#include <iostream>


class FlowCam {
public:
    FlowCam(){};
    FlowCam(const FlowCam&) = delete;            // no copy
    FlowCam& operator=(const FlowCam&) = delete; // no assign
    
    void setup(int in_capture_width, int in_capture_height, int in_screen_width, int in_screen_height, float zoom);
    
    void update(double delta_t);
    
    void drawDebug();
    
    void loadLUT(string path);
    
    cv::Size getFlowSize() const;
    
    void setFlowErosionSize(int in_flow_erosion_size);
    void setUseCamera(bool in_use_camera);
    void setScreenSize(int in_screen_width, int in_screen_height);
    void setZoom(float in_zoom);

    ofxCv::FlowFarneback opticalflow;
    ofxCv::ContourFinder contourfinder_low;
    ofxCv::ContourFinder contourfinder_high;

    cv::Mat frame_full, frame, frame_gray, frame_screen;
    cv::Mat magnitude, angle, flow, flow_low, flow_low_prev, flow_high, flow_high_prev, flow_behind, flow_new;
    cv::Mat flow_hist;
    
    ofImage frame_screen_im;

    float flow_threshold_low = 0.1f;
    float flow_threshold_high = 0.5f;
    const int pyrdown_steps = 2;

    
private:
    void updateFrame();
    void updateFlow();
    void doCapture();
    void computeRoi();
    void applyLUT();
    void reset();
    
    cv::Mat open_kernel;
    ofVideoPlayer video;
    ofVideoGrabber camera;
    cv::Rect capture_roi;
    
    double since_last_capture;
    
    int capture_width, capture_height, screen_width, screen_height, flow_width, flow_height;
    float zoom = 1.0;
    
    bool LUTloaded = false;
	ofVec3f lut[32][32][32];
    
    bool use_camera = true;
    int flow_erosion_size = 5;
};


#endif /* defined(__slow__flowcam__) */
