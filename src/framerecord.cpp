
#include "framerecord.h"


FrameRecord::FrameRecord(cv::Mat in_mask){
    current = 0;
    mask = in_mask;
    height = mask.rows;
    width = mask.cols;
}

FrameRecord::~FrameRecord(){
}

void FrameRecord::grab(cv::Mat frame, cv::Rect roi){
    cv::Mat selection = frame(roi);
    cv::resize(selection, selection, cv::Size(width, height));
    vector<cv::Mat> channels;
    cv::split(selection, channels);
    channels.push_back(mask);
    cv::merge(channels, selection);
    frames.push_back(selection);
}

void FrameRecord::update(){
    current = (current + 1) % frames.size();
}

void FrameRecord::draw(float x, float y, float width, float height){
    ofImage to_draw;
    ofxCv::toOf(frames[current], to_draw);
    to_draw.update();
    to_draw.draw(x, y, width, height);
}

void FrameRecord::draw(ofRectangle rect){
    draw(rect.x, rect.y, rect.width, rect.height);
}