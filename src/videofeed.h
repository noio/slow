#ifndef MOVE_VIDEOFEED_H_
#define MOVE_VIDEOFEED_H_

#include "ofMain.h"
#include "ofxCv.h"

using namespace std;
using namespace cv;
using namespace ofxCv;

enum VideoFeedWebcamResolution {
    WEBCAM_RES_1080,
    WEBCAM_RES_720,
    WEBCAM_RES_480
};

class VideoFeed : public ofThread
{

public:
    VideoFeed() {};
    VideoFeed(const VideoFeed&) = delete;            // no copy
    VideoFeed& operator=(const VideoFeed&) = delete; // no assign

    void setAspectRatio(int in_width, int in_height)
    {
        width = in_width;
        height = in_height;
    }

    void setFlip(int in_flip)
    {
        flip = in_flip;
    }
    
    void setMaxFps(float in_fps){
        wait_millis = 1000.0 / in_fps;
    }

    bool processFrame()
    {
        if (!needs_processing)
        {
            return false;
        }
        lock();
        frame_im.setFromPixels(pixels);
        unlock();
        if (!frame_im.isAllocated())
        {
            frame = cv::Mat::zeros(width, height, CV_8UC3);
            return false;
        }
        int capture_width = frame_im.getWidth();
        int capture_height = frame_im.getHeight();
        float ratio = (float)width / height;
        int w = min(capture_width, static_cast<int>(round(capture_height * ratio)));
        int h = min(capture_height, static_cast<int>(round(capture_width / ratio)));
        roi = cv::Rect((capture_width - w) / 2, (capture_height - h) / 2, w, h);
        frame_mat = toCv(frame_im);
        if (flip < 2)
        {
            cv::flip(frame_mat(roi), frame, flip);
        }
        else
        {
            frame = frame_mat(roi).clone();
        }
        frame_timestamp = ofGetElapsedTimeMillis();
        needs_processing = false;
        return true;
    }

    /*
     * Puts the latest frame into the given matrix,
     * and returns whether this a fresher frame than
     * the last time you called getMatrix.
     * Therefore, if you depend on the return, use only
     * once per loop.
     */
    bool getFrame(Mat& output_frame)
    {
        processFrame();
        output_frame = frame;
        bool frame_is_fresh = frame_timestamp > last_frame_returned;
        last_frame_returned = frame_timestamp;
        return frame_is_fresh;
    }

    void draw(float x, float y, float w, float h)
    {
        processFrame();
        if (frame_im.isAllocated())
        {
            if (flip == 1)
            {
                frame_im.drawSubsection(w, 0, -w, h, roi.x, roi.y, roi.width, roi.height);
            }
            else if (flip == 0)
            {
                frame_im.drawSubsection(0, h, w, -h, roi.x, roi.y, roi.width, roi.height);
            }
            else if (flip == -1)
            {
                frame_im.drawSubsection(w, h, -w, -h, roi.x, roi.y, roi.width, roi.height);
            }
            else
            {
                frame_im.drawSubsection(0, 0, w, h, roi.x, roi.y, roi.width, roi.height);
            }
        }
    }

protected:
    int width, height;
    char flip = 1;
    float wait_millis = 1000.0f / 30.0f;
    bool needs_processing = false;
    unsigned long long frame_timestamp = 0;
    unsigned long long last_frame_returned = 0;
    Mat frame_mat;
    Mat frame;
    ofImage frame_im;
    ofPixels pixels;
    cv::Rect roi;
};

class VideoFeedStatic : public VideoFeed
{
public:
    void setup(string path) ;
};

class VideoFeedWebcam : public VideoFeed
{
public:
    ~VideoFeedWebcam() { ofLogVerbose("VideoFeedImageWebcam") << "destroying"; waitForThread(true); }
    void setup(int device, VideoFeedWebcamResolution res) ;
    void threadedFunction() ;
private:
    ofVideoGrabber camera;
};

class VideoFeedImageUrl : public VideoFeed
{
public:
    ~VideoFeedImageUrl() { waitForThread(true); }
    void setup(string in_url) ;
    void threadedFunction() ;
private:
    string url;
    ofImage loader;
    int fail_count;

};



#endif /* defined(MOVE_VIDEOFEED_H_) */