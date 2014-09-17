#pragma once

#include "fileloader.h"
#include "ofMain.h"
#include "ofxJSON.h"

using namespace std;

namespace ofxDS{

typedef struct Skeleton
{
    int location;
    ofPoint head;
    ofPoint hand_left;
    ofPoint hand_right;
    ofPoint shoulder_center;
} Skeleton;

class SkeletonFeed

    : public ofThread
{

public:
    SkeletonFeed() {};
    SkeletonFeed(const SkeletonFeed&) = delete;            // no copy
    SkeletonFeed& operator=(const SkeletonFeed&) = delete; // no assign
    ~SkeletonFeed() { waitForThread(true); }


    static const int HEAD;
    static const int SHOULDER_CENTER;
    static const int SPINE;
    static const int HAND_LEFT;
    static const int HAND_RIGHT;

    void setMaxFPS(float in_fps);
    void setInputScale(float scale_x, float scale_y);
    void setInputScale(ofPoint scale);
    void setOutputScaleAndOffset(ofPoint scale, ofPoint offset);
    void setOutputFillScreen();

    vector<Skeleton> getSkeletons();

    void setup(string in_url);

    void drawDebug();

    void threadedFunction();

private:
    ofPoint getPoint(Json::Value point);

    vector<Skeleton> skeletons;
    FileLoaderSession loader;
    ofxJSONElement json;

    ofPoint input_scale, output_scale, output_offset;
    string url;
    float wait_millis = 1000.0f / 30.0f;
};
    
}