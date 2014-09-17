#pragma once

#include "fileloader.h"
#include "ofMain.h"
#include "ofxCv.h"

namespace ofxDS
{

enum VideoFeedWebcamResolution
{
    WEBCAM_RES_1080,
    WEBCAM_RES_720,
    WEBCAM_RES_480
};

template<typename PixelType>
class VideoFeed_ : public ofThread
{

public:
    VideoFeed_() {};
    VideoFeed_(const VideoFeed_&) = delete;            // no copy
    VideoFeed_& operator=(const VideoFeed_&) = delete; // no assign

    void setAspectRatio(int in_width, int in_height);
    void setFlip(char in_flip);
    void setMaxFPS(float in_fps);
    bool processFrame();

    /*
     * Puts the latest frame into the given matrix,
     * and returns whether this a fresher frame than
     * the last time you called getMatrix.
     * Therefore, if you depend on the return, use only
     * once per loop.
     */
    bool getFrame(cv::Mat& output_frame);
    void draw(float x, float y, float w, float h);

protected:
    int width = 640, height = 480;
    char flip = 1;
    float wait_millis = 1000.0f / 30.0f;
    bool needs_processing = false;
    unsigned long long frame_timestamp = 0;
    unsigned long long last_frame_returned = 0;
    cv::Mat frame_mat;
    cv::Mat frame;
    ofImage_<PixelType> frame_im;
    ofPixels_<PixelType> pixels;
    cv::Rect roi;
};

typedef VideoFeed_<unsigned char> VideoFeed;
typedef VideoFeed_<unsigned short> VideoFeed16Bit;

class VideoFeedStatic : public VideoFeed
{
public:
    void setup(string path) ;
};

class VideoFeedWebcam : public VideoFeed
{
public:
    ~VideoFeedWebcam() { waitForThread(true); }
    void setup(int device, VideoFeedWebcamResolution res) ;
    void threadedFunction() ;
private:
    ofVideoGrabber camera;
};

template<typename PixelType>
class VideoFeedImageURL_ : public VideoFeed_<PixelType>
{
public:
    ~VideoFeedImageURL_() { this->waitForThread(true); }
    void setup(string in_url) ;
    void threadedFunction() ;
    std::string getURL() const {return url;};
private:
    string url;
    
    ofPixels_<PixelType> pix_load;
    FileLoaderSession loader;
    int fail_count;

};

typedef VideoFeedImageURL_<unsigned char> VideoFeedImageURL;
typedef VideoFeedImageURL_<unsigned short> VideoFeed16BitImageURL;
}



