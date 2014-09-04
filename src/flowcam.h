
#ifndef MOVE_FLOWCAM_H_
#define MOVE_FLOWCAM_H_

#include "ofMain.h"
#include "ofxCv.h"

#include <iostream>

using namespace std;
using namespace cv;
using namespace ofxCv;

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
    Mat getFlow() const { return flow; }
    Mat getFlowHigh() const { return flow_high; };
    Mat getFLowHighHist() const { return flow_high_hist; };
    ofVec2f getFlowAt(float x, float y) const { return toOf(flow.at<Vec2f>(y, x)); };
    ofVec2f getFlowAtUnitPos(float x, float y) const { return toOf(flow.at<Vec2f>( y * flow.rows, x * flow.cols) ); };
    const vector<ofPolyline>& getContoursHigh() const { return contourfinder_high.getPolylines(); };
    const vector<ofPolyline>& getContoursLow() const { return contourfinder_low.getPolylines(); };

    bool hasData() { return has_data; };
    
    float flow_threshold_low = 0.1f;
    float flow_threshold_high = 0.5f;
    int flow_erosion_size = 5;

private:
    void reset();

    ContourFinder contourfinder_low, contourfinder_high;
    FlowFarneback opticalflow;

    Mat frame_gray, frame_screen;
    Mat magnitude, angle, flow, flow_low, flow_high, flow_high_hist;

    float global_flow;
    int flow_creep_counter = 0;

    int max_flow_width = 240;

    float last_update = 0;
    bool has_data = false;
};


#endif /* defined(MOVE_FLOWCAM_H_) */
