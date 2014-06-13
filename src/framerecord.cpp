
#include "framerecord.h"

FrameRecord::FrameRecord()
{
    current = 0;
    width = 100;
    height = 100;
    mask = cv::Mat(height, width, CV_8UC1, cv::Scalar(255));
}

FrameRecord::FrameRecord(cv::Mat in_mask)
{
    current = 0;
    mask = in_mask;
    height = mask.rows;
    width = mask.cols;
}

FrameRecord::~FrameRecord()
{
}

void FrameRecord::grab(cv::Mat frame, cv::Rect roi, float pad)
{
    cv::Mat selection = frame(roi);
    cv::copyMakeBorder(selection, selection, roi.height * pad, roi.height * pad, roi.width * pad, roi.width * pad, cv::BORDER_REPLICATE);
    cv::resize(selection, selection, cv::Size(width, height));
    vector<cv::Mat> channels;
    cv::split(selection, channels);
    channels.push_back(mask);
    cv::merge(channels, selection);
    frames.push_back(selection);
}

void FrameRecord::update(double delta_t)
{
    if (frames.size()) {
        current = (current + 1) % frames.size();
    }
}

void FrameRecord::draw(float x, float y, float width, float height)
{
    if (frames.size()) {
        ofImage to_draw;
        ofxCv::toOf(frames[current], to_draw);
        to_draw.update();
        to_draw.draw(x, y, width, height);
    }
}

void FrameRecord::draw(ofRectangle rect)
{
    draw(rect.x, rect.y, rect.width, rect.height);
}