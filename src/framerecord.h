
#ifndef SLOW_FRAMERECORD_H_
#define SLOW_FRAMERECORD_H_

#include "ofMain.h"
#include "ofxCv.h"

#include <vector>
#include <iostream>

using std::vector;

class FrameRecord {
public:
    FrameRecord();
    FrameRecord(cv::Mat mask);
    ~FrameRecord();
    FrameRecord(const FrameRecord&) = delete;            // no copy
    FrameRecord& operator=(const FrameRecord&) = delete; // no assign
    
    void grab(cv::Mat frame, cv::Rect roi, float pad = 1.0);
    void update(double delta_t);
    void draw(float x, float y, float width, float height);
    void draw(ofRectangle rect);
    
private:
    unsigned int height, width;
    cv::Mat mask;
    unsigned int current;
    vector<cv::Mat> frames;
    
};

#endif /* defined(SLOW_FRAMERECORD_H_) */
