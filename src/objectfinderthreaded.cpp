
#include "objectfinderthreaded.h"

void ObjectFinderThreaded::setup(std::string haarcascade)
{
    // Set up face detection
    objectfinder.setup(haarcascade);//
    objectfinder.setRescale(1.0); // Don't rescale internally because we'll feed it a small frame
    objectfinder.setMinNeighbors(2);
    objectfinder.setMultiScaleFactor(1.3);
    objectfinder.setFindBiggestObject(true);
    objectfinder.getTracker().setSmoothingRate(0.2);
}

bool ObjectFinderThreaded::startDetection(const cv::Mat input, const cv::Rect roi, const float& min_scale, const float& max_scale)
{
    if(!lock()) {
        return false;
    }

    objectfinder.setMinSizeScale(min_scale);
    objectfinder.setMaxSizeScale(max_scale);
    input_image = input(roi).clone();
    unlock();
    startThread(false);
    return true;
}

bool ObjectFinderThreaded::getResults(int& num_found, ofRectangle& first)
{
    if (!lock()) {
        return false;
    }

    num_found = objectfinder.size();

    if (num_found > 0) {
        first = objectfinder.getObjectSmoothed(0);
    }
    unlock();
    return true;
}

void ObjectFinderThreaded::threadedFunction()
{
    lock();
    objectfinder.update(input_image);
    ofLogVerbose("ObjectFinderThreaded") << "finished object detection";
    unlock();
}