#include "skeletonfeed.h"

using namespace ofxDS;

const int SkeletonFeed::SPINE = 1;
const int SkeletonFeed::SHOULDER_CENTER = 2;
const int SkeletonFeed::HEAD = 3;
const int SkeletonFeed::HAND_LEFT = 7;
const int SkeletonFeed::HAND_RIGHT = 11;


void SkeletonFeed::setMaxFPS(float in_fps)
{
    lock();
    wait_millis = 1000.0 / in_fps;
    unlock();
}

void SkeletonFeed::setInputScale(float scale_x, float scale_y)
{
    lock();
    input_scale = ofPoint(scale_x, scale_y, 1.0);
    unlock();
}

void SkeletonFeed::setInputScale(ofPoint scale)
{
    lock();
    input_scale = scale;
    unlock();
}

/*
 * Set the top-left point and size of the rectangle that the skeletons
 * are projected to
 */
void SkeletonFeed::setOutputScaleAndOffset(ofPoint scale, ofPoint offset)
{
    lock();
    output_scale = scale;
    output_offset = offset;
    ofLogVerbose("SkeletonFeed") << "set offset " << output_offset << ", scale " << output_scale;
    unlock();
}

/*
 * This assumes a skeleton feed that, like on Kinect, maps 1:1 to the RGB
 * pixels. Otherwise, use setOutputScaleAndOffset() to manually set projection.
 */
void SkeletonFeed::setOutputFillScreen()
{
    float ratio = input_scale.x / input_scale.y;
    int w = MAX(ofGetWidth(), ofGetHeight() * ratio);
    int h = MAX(ofGetHeight(), ofGetWidth() / ratio);
    setOutputScaleAndOffset(ofPoint(w, h, 1),
                            ofPoint((ofGetWidth() - w) / 2, (ofGetHeight() - h) / 2, 1));
}

void SkeletonFeed::setup(string in_url)
{
    url = in_url;
    setInputScale(640, 480);
    setOutputFillScreen();
    loader.setup(in_url);
    startThread(true, false);
}

ofPoint SkeletonFeed::getPoint(Json::Value point)
{
    return ofPoint(point["X"].asFloat(),
                   point["Y"].asFloat(),
                   point["Z"].asFloat()) / input_scale * output_scale + output_offset;
}


void SkeletonFeed::threadedFunction()
{
    while (isThreadRunning())
    {
        double fetch_start = ofGetElapsedTimeMillis();
        if (!json.parse(loader.loadURL(url).data))
        {
            ofLogWarning("SkeletonFeed") << "load fail";
            ofSleepMillis(1000);
        }
        else
        {
            if (json.isMember("Skeletons") && json["Skeletons"].isArray())
            {
                lock();
                skeletons.resize(json["Skeletons"].size());
                for (int i = 0; i < json["Skeletons"].size(); i ++)
                {
                    Json::Value joints = json["Skeletons"][i]["Skeleton"]["Joints"];
                    skeletons[i].location = json["Skeletons"][i]["Location"].asInt();
                    skeletons[i].head = getPoint(joints[HEAD]);
                    skeletons[i].hand_left = getPoint(joints[HAND_LEFT]);
                    skeletons[i].hand_right = getPoint(joints[HAND_RIGHT]);
                    skeletons[i].shoulder_center = getPoint(joints[SHOULDER_CENTER]);
                }
                unlock();
            }
        }
        double elapsed = ofGetElapsedTimeMillis() - fetch_start;
        double wait = wait_millis - elapsed;
        if (wait > 0)
        {
            ofSleepMillis(wait);
        }
    }
}

vector<Skeleton> SkeletonFeed::getSkeletons()
{
    Poco::ScopedLock<ofMutex> lock(mutex);
    return skeletons;
}

void SkeletonFeed::drawDebug()
{
    ofSetLineWidth(2.0f);
    ofNoFill();
    lock();
    for (int i = 0; i < skeletons.size(); i++)
    {
        ofSetColor(ofColor::fromHsb(190 * i, 255, 255), 255);
        ofPoint hd = skeletons[i].head;
        ofPoint hl = skeletons[i].hand_left;
        ofPoint hr = skeletons[i].hand_right;
        ofDrawBitmapStringHighlight(ofToString(i), hd);
        ofSetCircleResolution(6);
        ofCircle(hd, 20);
        ofDrawBitmapStringHighlight(ofToString(i), hl);
        ofSetCircleResolution(4);
        ofCircle(hl, 20);
        ofDrawBitmapStringHighlight(ofToString(i), hr);
        ofSetCircleResolution(4);
        ofCircle(hr, 20);
    }
    unlock();
    ofSetColor(ofColor::white);
}