
#ifndef SLOW_FLOWCAM_H_
#define SLOW_FLOWCAM_H_

#include "ofMain.h"
#include "ofxCv.h"

#include <iostream>

using std::string;

class FlowCam : public ofThread {
public:
    FlowCam(){};
    FlowCam(const FlowCam&) = delete;            // no copy
    FlowCam& operator=(const FlowCam&) = delete; // no assign
    
    void setup(int in_capture_width, int in_capture_height, int in_screen_width, int in_screen_height, float zoom);
    void update();
    void draw(float x, float y, float width, float height);
    
    void drawDebug();
    
    void loadLUT(string path);
    
    float getFlowThresholdLow() const {return flow_threshold_low; };
    cv::Size getFlowSize() const { return cv::Size(flow_width, flow_height); };
    cv::Size getFrameSize() const { return cv::Size(frame.cols, frame.rows); };
    cv::Mat getFlowHigh() const { return flow_high; };
    cv::Mat getFlowLow() const { return flow_low; };
    ofVec2f getFlowAt(float x, float y) { return ofxCv::toOf(flow.at<cv::Vec2f>(y, x)); };
    cv::Mat getFrame() const { return frame; };
    const vector<ofPolyline>&  getContoursHigh() const { return contourfinder_high.getPolylines(); };
    const vector<ofPolyline>&  getContoursLow() const { return contourfinder_low.getPolylines(); };

    void setFlowThreshold(float threshold_low, float threshold_high);
    void setFlowErosionSize(int in_flow_erosion_size);
    void setScreenSize(int in_screen_width, int in_screen_height);
    void setCaptureSize(int in_capture_width, int in_capture_height);
    void setZoom(float in_zoom);
    void setFlip(int flip);
    bool hasData(){return has_data;};
    
    const int pyrdown_steps = 3;

    
private:
    void initGrabber();
    void threadedFunction();
    void updateFrame();
    void updateFlow();
    void computeRoi();
    void applyLUT();
    void reset();
    
    cv::Mat open_kernel;
    ofVideoGrabber camera;
    cv::Rect capture_roi;
    
    ofxCv::FlowFarneback opticalflow;
    ofxCv::ContourFinder contourfinder_low;
    ofxCv::ContourFinder contourfinder_high;
    
    cv::Mat frame_full, frame, frame_gray, frame_screen;
    cv::Mat magnitude, angle, flow, flow_low, flow_low_prev, flow_high, flow_high_prev; // flow_behind, flow_new;
//    cv::Mat flow_hist;
    
    ofImage frame_screen_im;
    
    float last_capture;
    
    int capture_width, capture_height, screen_width, screen_height, flow_width, flow_height;
    float zoom = 1.0;
    int flip = 1;
    float flow_threshold_low = 0.1f;
    float flow_threshold_high = 0.5f;
    int flow_erosion_size = 5;
    
    bool has_data = false;
    bool LUTloaded = false;
	ofVec3f lut[32][32][32];
};


#endif /* defined(__slow__flowcam__) */
