#include "flowcam.h"

using namespace ofxCv;

void FlowCam::setup(int max_flow_width)
{
    // Initialize member variables
    setFlowErosionSize(5);
    // OpticalFlow setup
    opticalflow.setPyramidScale(0.5);
    opticalflow.setNumLevels(2);
    opticalflow.setWindowSize(7);
    opticalflow.setNumIterations(3);
    opticalflow.setPolyN(5);
    opticalflow.setPolySigma(1.2);
    //
    contourfinder_low.setSimplify(true);
    contourfinder_low.setMinArea(80);
    contourfinder_low.getTracker().setSmoothingRate(0.2);
    contourfinder_high.setSimplify(true);
    contourfinder_high.setMinArea(80);
    contourfinder_high.getTracker().setSmoothingRate(0.2);
    //
    flow = cv::Mat::zeros(2, 2, CV_32FC2);
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
    ofxCv::drawMat(flow_high, 0, 0, ofGetWidth(), ofGetHeight());
    ofDisableBlendMode();
    ofPushMatrix();
    ofSetLineWidth(4.0);
    ofSetColor(ofColor::red);
    ofScale(ofGetWidth() / (float)flow_high.cols, ofGetHeight() / (float)flow_high.rows);
    ofPopMatrix();
    ofPopStyle();
}

void FlowCam::setFlowErosionSize(int in_flow_erosion_size)
{
    flow_erosion_size = in_flow_erosion_size;
    open_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                            cv::Size(2 * flow_erosion_size + 1, 2 * flow_erosion_size + 1),
                                            cv::Point(flow_erosion_size, flow_erosion_size));
}


////////// PRIVATE METHODS //////////

void FlowCam::update(cv::Mat frame)
{
    float delta_t = ofGetElapsedTimef() - last_update;
    cv::cvtColor(frame, frame_gray, CV_BGR2GRAY);
    while (frame_gray.cols > max_flow_width)
    {
        cv::pyrDown(frame_gray, frame_gray);
    }
    opticalflow.calcOpticalFlow(frame_gray);
    flow = opticalflow.getFlow();
    // ofxCV wrapper returns a 1x1 flow image after the first optical flow computation.
    if (frame_gray.cols != flow.cols || frame_gray.rows != flow.rows)
    {
        ofLogVerbose("FlowCam") << "Initialize flow data";
        flow = cv::Mat::zeros(frame_gray.rows, frame_gray.cols, CV_32FC2);
        opticalflow.resetFlow();
        opticalflow.calcOpticalFlow(frame_gray);
    }
    //
    std::vector<cv::Mat> xy(2);
    cv::split(flow, xy);
    cv::cartToPolar(xy[0], xy[1], magnitude, angle, true);
    //
    // Low speed mask
    flow_low = magnitude > flow_threshold_low; // & magnitude < adj_flow_threshold_high;
    cv::dilate(flow_low, flow_low, open_kernel);
    cv::erode(flow_low, flow_low, open_kernel);
    cv::erode(flow_low, flow_low, open_kernel);
    //
    // Compute the high speed mask
    flow_high = magnitude > flow_threshold_high;
    cv::erode(flow_high, flow_high, open_kernel);
    cv::dilate(flow_high, flow_high, open_kernel);
    // Update history
    if (flow_high_hist.size() != flow_high.size())
    {
        flow_high_hist = cv::Mat::zeros(flow_high.rows, flow_high.cols, CV_8U);
    }
    flow_high_hist += flow_high * (delta_t * 2);
    flow_high_hist -= 1;
    ofxCv::blur(flow_high_hist, flow_high_hist, 3);
    //
    contourfinder_high.findContours(flow_high);
    contourfinder_low.findContours(flow_low);
    //
    // Check for flow creep
    global_flow = cv::sum(flow_high)[0] / 255 / (float)(flow_high.cols * flow_high.rows);
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
    has_data = true;
    last_update = ofGetElapsedTimef();
}
