
#ifndef SLOW_OBJECTFINDERTHREADED_H_
#define SLOW_OBJECTFINDERTHREADED_H_

#include "ofMain.h"
#include "ofxCv.h"
#include <iostream>

class ObjectFinderThreaded : public ofThread {
    
public:
    void threadedFunction();
    
    void setup(std::string haarcascade);
    bool startDetection(const cv::Mat input, const cv::Rect roi, const float& min_scale, const float& max_scale);
    bool getResults(int& num_found, ofRectangle& first);
    
private:
    ofxCv::ObjectFinder objectfinder;
    cv::Mat input_image;
};

#endif /* defined(SLOW_OBJECTFINDERTHREADED_H_) */
