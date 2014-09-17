
#ifndef MOVE_FLOWCAM_H_
#define MOVE_FLOWCAM_H_

#include "ofMain.h"
#include "ofxCv.h"

#include <iostream>

namespace ofxDS
{

class FlowCam
{
public:
    FlowCam() {};
    FlowCam(const FlowCam&) = delete;            // no copy
    FlowCam& operator=(const FlowCam&) = delete; // no assign

    void setup(int max_flow_width);
    void update(cv::Mat frame);

    void drawDebug();

    ofPoint getSize() const { return ofPoint(frame_gray.cols, frame_gray.rows); };
    cv::Mat getFlow() const { return flow; }
    cv::Mat getFlowHigh() const { return flow_high; };
    cv::Mat getFLowHighHist() const { return flow_high_hist; };
    const std::vector<ofPolyline>& getContoursHigh() const { return contourfinder.getPolylines(); };
    ofVec2f getFlowAt(float x, float y) const { return ofxCv::toOf(flow.at<cv::Vec2f>(y, x)); };
    ofVec2f getFlowAtUnitPos (const ofPoint p) const { return getFlowAtUnitPos(p.x, p.y); };
    ofVec2f getFlowAtUnitPos (float x, float y) const {
            return ofxCv::toOf(flow.at<cv::Vec2f>(
            MAX(0, MIN(flow.rows - 1, y * flow.rows)),
            MAX(0, MIN(flow.cols - 1, x * flow.cols))));
        };

    bool hasData() const { return has_data; };

    float flow_threshold_low = 0.1f;
    float flow_threshold_high = 0.5f;
    int flow_erosion_size = 5;

private:
    void reset();

    ofxCv::ContourFinder contourfinder;
    ofxCv::FlowFarneback opticalflow;

    cv::Mat frame_gray, frame_screen;
    cv::Mat magnitude, angle, flow, flow_low, flow_high, flow_high_hist;

    float global_flow;
    int flow_creep_counter = 0;

    int max_flow_width = 240;

    float last_update = 0;
    bool has_data = false;
};

}

#endif /* defined(MOVE_FLOWCAM_H_) */

