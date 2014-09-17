#include "videofeed.h"
#include "utilities.h"

using namespace ofxDS;
using namespace std;
using namespace cv;
using namespace ofxCv;

template<typename PixelType>
void VideoFeed_<PixelType>::setAspectRatio(int in_width, int in_height)
{
    lock();
    width = MAX(1, in_width);
    height = MAX(1, in_height);
    unlock();
}

template<typename PixelType>
void VideoFeed_<PixelType>::setFlip(char in_flip)
{
    flip = in_flip;
}

template<typename PixelType>
void VideoFeed_<PixelType>::setMaxFPS(float in_fps){
    lock();
    wait_millis = 1000.0 / in_fps;
    unlock();
}

template<typename PixelType>
bool VideoFeed_<PixelType>::processFrame()
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
template<typename PixelType>
bool VideoFeed_<PixelType>::getFrame(Mat& output_frame)
{
    processFrame();
    output_frame = frame;
    bool frame_is_fresh = frame_timestamp > last_frame_returned;
    last_frame_returned = frame_timestamp;
    return frame_is_fresh;
}

template<typename PixelType>
void VideoFeed_<PixelType>::draw(float x, float y, float w, float h)
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


void VideoFeedStatic::setup(string path)
{
    frame_im.loadImage(path);
    pixels = frame_im.getPixelsRef();
    needs_processing = true;
}


void VideoFeedWebcam::setup(int device, VideoFeedWebcamResolution res)
{
    Poco::ScopedLock<ofMutex> lock(mutex);
    waitForThread(true);
    if (camera.isInitialized())
    {
        camera.close();
    }
    camera.setDeviceID(device);
    int capture_width, capture_height;
    switch (res)
    {
        case WEBCAM_RES_480:
            capture_width = 640;
            capture_height = 480;
            break;

        case WEBCAM_RES_720:
            capture_width = 1280;
            capture_height = 720;
            break;

        case WEBCAM_RES_1080:
            capture_width = 1920;
            capture_height = 1080;
            break;

        default:
            break;
    }
    camera.initGrabber(capture_width, capture_height, false);
    setFlip(1);
    startThread(true, false);
}

void VideoFeedWebcam::threadedFunction()
{
    while (isThreadRunning())
    {
        if (camera.isInitialized())
        {
            double fetch_start = ofGetElapsedTimeMillis();
            camera.update();
            if (camera.isFrameNew())
            {
                lock();
                pixels = camera.getPixelsRef();
                needs_processing = true;
                unlock();
                double wait = wait_millis - (ofGetElapsedTimeMillis() - fetch_start);
                if (wait > 0)
                {
                    ofSleepMillis(wait);
                }
            }
        }
    }
}


template<typename PixelType>
void VideoFeedImageURL_<PixelType>::setup(string in_url)
{
    url = in_url;
    fail_count = 0;
    this->setFlip(2);
    loader.setup(url);
    this->startThread(true, false);
}

template<typename PixelType>
void VideoFeedImageURL_<PixelType>::threadedFunction()
{
    while (this->isThreadRunning())
    {
        double fetch_start = ofGetElapsedTimeMillis();
        if (!ofLoadImage(pix_load, loader.loadURL(url).data))
        {
            ofLogWarning("VideoFeedImageURL") << "load fail (" << fail_count << ") " << url;
            // When loading fails, the ofImage resets bUseTexture to true
            fail_count ++;
            ofSleepMillis(1000);
        }
        else
        {
            this->lock();
            this->pixels.swap(pix_load);
            this->unlock();
            this->needs_processing = true;
            fail_count = 0;
            double wait = this->wait_millis - (ofGetElapsedTimeMillis() - fetch_start);
            if (wait > 0)
            {
                ofSleepMillis(wait);
            }
        }
    }
}

template class VideoFeed_<unsigned char>;
template class VideoFeed_<unsigned short>;

template class VideoFeedImageURL_<unsigned char>;
template class VideoFeedImageURL_<unsigned short>;
