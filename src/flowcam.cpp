#include "flowcam.h"
#include "utilities.h"

using namespace ofxCv;

void FlowCam::setup(int in_capture_width, int in_capture_height, int in_screen_width, int in_screen_height, float zoom)
{
    // Initialize the camera first, before we do other stuff that depends on having a frame
    // Use an external camera if one is connected
    if (camera.listDevices().size() > 1) {
        camera.setDeviceID(1);
    }

    // Camera and video grabber
    if (camera.isInitialized()) {
        camera.close();
    }

    camera.initGrabber(in_capture_width, in_capture_height);
    video.loadMovie("videos/damrak/damrak_3.mov");
    video.setVolume(0);
    video.play();
    // Initialize member variables
    capture_width = in_capture_width;
    capture_height = in_capture_height;
    setScreenSize(in_screen_width, in_screen_height);
    setUseCamera(use_camera);
    setZoom(zoom);
    setFlowErosionSize(flow_erosion_size);
    // Contourfinder setup
    contourfinder_high.setSimplify(true);
    contourfinder_high.setMinArea(80);
    contourfinder_high.getTracker().setSmoothingRate(0.2);
    contourfinder_low.setSimplify(true);
    contourfinder_low.setMinArea(80);
    contourfinder_low.getTracker().setSmoothingRate(0.2);
    // OpticalFlow setup
    opticalflow.setPyramidScale(0.5);
    opticalflow.setNumLevels(2);
    opticalflow.setWindowSize(7);
    opticalflow.setNumIterations(3);
    opticalflow.setPolyN(5);
    opticalflow.setPolySigma(1.2);
    //
    reset();
}

void FlowCam::reset(){
    opticalflow.resetFlow();
}

void FlowCam::update(double delta_t)
{
    since_last_capture += delta_t;
    doCapture();

    if ((use_camera && camera.isFrameNew()) || (!use_camera && video.isFrameNew()) || frame.empty() ) {
        updateFrame();
        updateFlow();
        since_last_capture = 0;
    }
}


void FlowCam::updateFrame()
{
    cv::flip(frame_full(capture_roi), frame, 1);
    cv::cvtColor(frame, frame_gray, CV_BGR2GRAY);
    cv::pyrDown(frame_gray, frame_gray);
    cv::pyrDown(frame_gray, frame_gray);
}

void FlowCam::updateFlow()
{
    opticalflow.calcOpticalFlow(frame_gray);
    flow = opticalflow.getFlow();
    std::swap(flow_low_prev, flow_low); // TODO: Remove this whole flow_low_prev thing?
    std::swap(flow_high_prev, flow_high);

    // ofxCV wrapper returns a 1x1 flow image after the first optical flow computation.
    if (flow.cols == 1) {
        flow_low_prev = cv::Mat::zeros(frame_gray.rows, frame_gray.cols, CV_8U);
        flow_high_prev = cv::Mat::zeros(frame_gray.rows, frame_gray.cols, CV_8U);
        flow = cv::Mat::zeros(frame_gray.rows, frame_gray.cols, CV_32FC2);
    }

    std::vector<cv::Mat> xy(2);
    cv::split(flow, xy);
    cv::cartToPolar(xy[0], xy[1], magnitude, angle, true);
    //
    // Compute the low speed mask
    // Shadow these variables:
    float adj_flow_threshold_low = flow_threshold_low * since_last_capture * 30.0;
    float adj_flow_threshold_high = flow_threshold_high * since_last_capture * 30.0;
    flow_low = magnitude > adj_flow_threshold_low & magnitude < adj_flow_threshold_high;
    cv::erode(flow_low, flow_low, open_kernel);
    cv::dilate(flow_low, flow_low, open_kernel);
    cv::dilate(flow_low, flow_low, open_kernel);
    cv::erode(flow_low, flow_low, open_kernel);
    //
    // Compute the high speed mask
    flow_high = magnitude > adj_flow_threshold_high; // & flow_low_prev > 0;
    cv::erode(flow_high, flow_high, open_kernel);
    cv::dilate(flow_high, flow_high, open_kernel);
    flow_behind = flow_low_prev & (255 - flow_low);
    flow_new = flow_high & ( 255 - flow_high_prev);
    contourfinder_low.findContours(flow_low);
    contourfinder_high.findContours(flow_high);
}

void FlowCam::drawDebug()
{
    ofPushStyle();
    // Draw the optical flow maps
    ofEnableBlendMode(OF_BLENDMODE_ADD);
    ofSetColor(224, 160, 58, 128);
    ofxCv::drawMat(flow_low, 0, 0, ofGetWidth(), ofGetHeight());
    ofDisableBlendMode();
    ofPushMatrix();
    ofSetLineWidth(4.0);
    ofSetColor(ofColor::red);
    ofScale(ofGetWidth() / (float)flow_high.cols, ofGetHeight() / (float)flow_high.rows);

    for (int i = 0; i < contourfinder_high.size(); i ++) {
        contourfinder_high.getPolyline(i).draw();
    }

    ofSetColor(ofColor::green);

    for (int i = 0; i < contourfinder_low.size(); i ++) {
        cv::Rect r = contourfinder_low.getTracker().getSmoothed(contourfinder_low.getTracker().getLabelFromIndex(i));
//        contourfinder_low.getPolyline(i).getResampledBySpacing(30).draw();
        ofRect(toOf(r));
//        cv::RotatedRect a = contourfinder_low.getMinAreaRect(i);
    }

    ofPopMatrix();
    ofPopStyle();
}




void FlowCam::setUseCamera(bool in_use_camera)
{
    if (use_camera == in_use_camera) return;
    use_camera = in_use_camera;
    computeRoi();
}

void FlowCam::setScreenSize(int in_screen_width, int in_screen_height)
{
    if (screen_width == in_screen_width && screen_height == in_screen_height) return;
    screen_width = in_screen_width;
    screen_height = in_screen_height;
    computeRoi();
}

void FlowCam::setZoom(float in_zoom){
    if (zoom == in_zoom) return;
    zoom = in_zoom;
    computeRoi();
}

void FlowCam::setFlowErosionSize(int in_flow_erosion_size)
{
    flow_erosion_size = in_flow_erosion_size;
    open_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                            cv::Size(2 * flow_erosion_size + 1, 2 * flow_erosion_size + 1),
                                            cv::Point(flow_erosion_size, flow_erosion_size));
}


////////// PRIVATE METHODS //////////

void FlowCam::doCapture()
{
    if (use_camera) {
        camera.update();
        frame_full = toCv(camera);
    } else {
        video.update();
        frame_full = toCv(video.getPixelsRef());
        cv::resize(frame_full, frame_full, cv::Size(capture_width, capture_height));
    }
}

void FlowCam::computeRoi()
{
    doCapture(); // Get a single frame so we have the cam resolution
    float ratio = (float)screen_width / screen_height;
    int w = std::min(frame_full.cols, static_cast<int>(round(frame_full.rows * ratio)));
    int h = std::min(frame_full.rows, static_cast<int>(round(frame_full.cols / ratio)));
    w /= zoom;
    h /= zoom;
    capture_roi = cv::Rect((frame_full.cols - w) / 2, (frame_full.rows - h) / 2, w, h);
    reset();
}