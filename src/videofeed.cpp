#include "videofeed.h"

void VideoFeedStatic::setup(string path)
{
    frame_im.loadImage(path);
    pixels = frame_im.getPixelsRef();
    needs_processing = true;
}


void VideoFeedWebcam::setup(int device, int capture_width, int capture_height)
{
    Poco::ScopedLock<ofMutex> lock(mutex);
    waitForThread(true);
    if (camera.isInitialized())
    {
        camera.close();
    }
    camera.setDeviceID(device);
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

void VideoFeedImageUrl::setup(string in_url)
{
    url = in_url;
    fail_count = 0;
    setFlip(2);
    loader.setUseTexture(false);
    startThread(true, false);
}

void VideoFeedImageUrl::threadedFunction()
{
    while (isThreadRunning())
    {
        double fetch_start = ofGetElapsedTimeMillis();
        if (!loader.loadImage(url))
        {
            ofLogWarning("VideoFeedImageUrl") << "load fail (" << fail_count << ") " << url;
            // When loading fails, the ofImage resets bUseTexture to true
            loader.setUseTexture(false);
            fail_count ++;
            ofSleepMillis(fail_count < 60 ? 10 : 1000);
        }
        else
        {
            lock();
            pixels = loader.getPixelsRef();
            unlock();
            needs_processing = true;
            fail_count = 0;
            double wait = wait_millis - (ofGetElapsedTimeMillis() - fetch_start);
            if (wait > 0)
            {
                ofSleepMillis(wait);
            }
        }
    }
}