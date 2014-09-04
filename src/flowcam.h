
#ifndef MOVE_FLOWCAM_H_
#define MOVE_FLOWCAM_H_

#include "ofMain.h"
#include "ofxCv.h"

#include <iostream>

using namespace std;
using namespace cv;

class FlowCam
{
public:
    FlowCam() {};
    FlowCam(const FlowCam&) = delete;            // no copy
    FlowCam& operator=(const FlowCam&) = delete; // no assign

    void setup(int max_flow_width);
    void update(Mat frame);

    void drawDebug();

    ofPoint getSize() const { return ofPoint(frame_gray.cols, frame_gray.rows); };
    cv::Mat getFlow() const { return flow; }
    cv::Mat getFlowHigh() const { return flow_high; };
    cv::Mat getFLowHighHist() const { return flow_high_hist; };
    ofVec2f getFlowAt(float x, float y) const { return ofxCv::toOf(flow.at<cv::Vec2f>(y, x)); };
    ofVec2f getFlowAtUnitPos(float x, float y) const { return ofxCv::toOf(flow.at<cv::Vec2f>( y * flow.rows, x * flow.cols) ); };
    const vector<ofPolyline>& getContoursLow() const { return contourfinder_low.getPolylines(); };
    const vector<ofPolyline>& getContoursHigh() const { return contourfinder_high.getPolylines(); };

    void setFlowErosionSize(int in_flow_erosion_size);
    bool hasData() { return has_data; };
    
    float flow_threshold_low = 0.1f;
    float flow_threshold_high = 0.5f;

private:
    void reset();

    cv::Mat open_kernel;

    ofxCv::FlowFarneback opticalflow;
    
    ofxCv::ContourFinder contourfinder_low, contourfinder_high;

    cv::Mat frame_gray, frame_screen;
    cv::Mat magnitude, angle, flow, flow_low, flow_high, flow_high_hist;

    float global_flow;
    int flow_creep_counter = 0;

    int max_flow_width = 240;

    int flow_erosion_size;

    float last_update = 0;
    bool has_data = false;
};


#endif /* defined(MOVE_FLOWCAM_H_) */
