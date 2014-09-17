#include "flowcam.h"

using namespace ofxDS;
using namespace std;
using namespace cv;
using namespace ofxCv;

void FlowCam::setup(int max_flow_width)
{
    // OpticalFlow setup
    opticalflow.setPyramidScale(0.5);
    opticalflow.setNumLevels(2);
    opticalflow.setWindowSize(7);
    opticalflow.setNumIterations(3);
    opticalflow.setPolyN(5);
    opticalflow.setPolySigma(1.2);
    //
    //
    flow = Mat::zeros(2, 2, CV_32FC2);
    //
    reset();
}

void FlowCam::reset()
{
    flow_creep_counter = 0;
    opticalflow.resetFlow();
    has_data = false;
}

void FlowCam::drawDebug()
{
    ofPushStyle();
    // Draw the optical flow maps
    ofEnableBlendMode(OF_BLENDMODE_ADD);
    ofSetColor(224, 160, 58, 128);
    if (!flow_high.empty())
    {
        drawMat(flow_high, 0, 0, ofGetWidth(), ofGetHeight());
    }
    ofDisableBlendMode();
    ofPushMatrix();
    ofSetLineWidth(4.0);
    ofSetColor(ofColor::red);
    ofScale(ofGetWidth() / (float)flow_high.cols, ofGetHeight() / (float)flow_high.rows);
    ofPopMatrix();
    ofPopStyle();
}


////////// PRIVATE METHODS //////////

void FlowCam::update(Mat frame)
{
    Mat kernel = getStructuringElement(MORPH_ELLIPSE,
                                       cv::Size(2 * flow_erosion_size + 1, 2 * flow_erosion_size + 1),
                                       cv::Point(flow_erosion_size, flow_erosion_size));
    float delta_t = ofGetElapsedTimef() - last_update;
    if (frame.channels() > 1)
    {
        cvtColor(frame, frame_gray, CV_RGB2GRAY);
    }
    else
    {
        frame_gray = frame;
    }
    while (frame_gray.cols > max_flow_width)
    {
        pyrDown(frame_gray, frame_gray);
    }
    opticalflow.calcOpticalFlow(frame_gray);
    flow = opticalflow.getFlow();
    // ofxCV wrapper returns a 1x1 flow image after the first optical flow computation.
    if (frame_gray.cols != flow.cols || frame_gray.rows != flow.rows)
    {
        ofLogVerbose("FlowCam") << "Initialize flow data";
        flow = Mat::zeros(frame_gray.rows, frame_gray.cols, CV_32FC2);
        opticalflow.resetFlow();
        opticalflow.calcOpticalFlow(frame_gray);
    }
    //
    vector<Mat> xy(2);
    split(flow, xy);
    cartToPolar(xy[0], xy[1], magnitude, angle, true);
    //
    // Low speed mask
    flow_low = magnitude > flow_threshold_low; // & magnitude < adj_flow_threshold_high;
    dilate(flow_low, flow_low, kernel);
    erode(flow_low, flow_low, kernel);
    erode(flow_low, flow_low, kernel);
    //
    // Compute the high speed mask
    flow_high = magnitude > flow_threshold_high;
    dilate(flow_high, flow_high, kernel);
    erode(flow_high, flow_high, kernel);
    //
    contourfinder.findContours(flow_high);
    // Update history
    if (flow_high_hist.size() != flow_high.size())
    {
        flow_high_hist = Mat::zeros(flow_high.rows, flow_high.cols, CV_8U);
    }
    flow_high_hist += flow_high * (delta_t * 2);
    flow_high_hist -= 1;
    blur(flow_high_hist, flow_high_hist, 3);
    has_data = true;
    //
    // Check for flow creep
    global_flow = sum(flow_high)[0] / 255 / (float)(flow_high.cols * flow_high.rows);
    if (global_flow > 0.2f)
    {
        flow_creep_counter ++;
        if (flow_creep_counter > 100)
        {
            ofLogNotice("FlowCam") << "flow creep detected; resetting";
            reset();
        }
    }
    else
    {
        flow_creep_counter = MAX(0, flow_creep_counter - 1);
    }
    last_update = ofGetElapsedTimef();
}
