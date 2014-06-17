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

    // Initialize member variables
    setCaptureSize(in_capture_width, in_capture_height);
    setScreenSize(in_screen_width, in_screen_height);
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

void FlowCam::reset()
{
    opticalflow.resetFlow();
    has_data = false;
}

void FlowCam::update(double delta_t)
{
    since_last_capture += delta_t;

    camera.update();

    if (camera.isFrameNew()) {
        frame_full = toCv(camera);
        updateFrame();
        updateFlow();
        has_data = true;
        ofLogNotice("FlowCam") << round(1/since_last_capture) << " captures/s";
        since_last_capture = 0;
    }
}

void FlowCam::draw(float x, float y, float width, float height){
    frame_screen_im.draw(x, y, width, height);
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


void FlowCam::setScreenSize(int in_screen_width, int in_screen_height)
{
    if (screen_width == in_screen_width && screen_height == in_screen_height) return;

    screen_width = in_screen_width;
    screen_height = in_screen_height;
    computeRoi();
}

void FlowCam::setZoom(float in_zoom)
{
    if (zoom == in_zoom) return;

    zoom = in_zoom;
    computeRoi();
}

void FlowCam::setCaptureSize(int in_capture_width, int in_capture_height){
    if (capture_width == in_capture_width && capture_height == in_capture_height) return;
    capture_width = in_capture_width;
    capture_height = in_capture_height;
    // Camera and video grabber
    if (camera.isInitialized()) {
        camera.close();
    }
    camera.initGrabber(in_capture_width, in_capture_height);
    ofLogNotice("FlowCam") << "Camera set to " << camera.getWidth() << "x" << camera.getHeight();
    computeRoi();
}

void FlowCam::setFlowErosionSize(int in_flow_erosion_size)
{
    flow_erosion_size = in_flow_erosion_size;
    open_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                            cv::Size(2 * flow_erosion_size + 1, 2 * flow_erosion_size + 1),
                                            cv::Point(flow_erosion_size, flow_erosion_size));
}

void FlowCam::setFlowThreshold(float threshold_low, float threshold_high){
    flow_threshold_low = threshold_low;
    flow_threshold_high = threshold_high;
}


void FlowCam::loadLUT(string path)
{
    LUTloaded = false;
    ofFile file(path);
    string line;

    for(int i = 0; i < 5; i++) {
        getline(file, line);
        ofLog() << "Skipped line: " << line;
    }

    for(int z = 0; z < 32; z++) {
        for(int y = 0; y < 32; y++) {
            for(int x = 0; x < 32; x++) {
                ofVec3f cur;
                file >> cur.x >> cur.y >> cur.z;
                lut[x][y][z] = cur;
            }
        }
    }

    LUTloaded = true;
}


////////// PRIVATE METHODS //////////



void FlowCam::updateFrame()
{
    cv::flip(frame_full(capture_roi), frame, 1);
    cv::resize(frame, frame_screen, cv::Size(screen_width, screen_height), 0, 0, cv::INTER_NEAREST);
    toOf(frame_screen, frame_screen_im);
    cv::cvtColor(frame, frame_gray, CV_BGR2GRAY);

    for (int i = 0; i < pyrdown_steps; i++) {
        cv::pyrDown(frame_gray, frame_gray);
    }

    assert(frame_gray.cols == flow_width && frame_gray.rows == flow_height);
    frame_screen_im.update();
}

void FlowCam::updateFlow()
{
    opticalflow.calcOpticalFlow(frame_gray);
    flow = opticalflow.getFlow();
    std::swap(flow_low_prev, flow_low); // TODO: Remove this whole flow_low_prev thing?
    std::swap(flow_high_prev, flow_high);

    // ofxCV wrapper returns a 1x1 flow image after the first optical flow computation.
    if (flow.cols == 1) {
        flow = cv::Mat::zeros(frame_gray.rows, frame_gray.cols, CV_32FC2);
        flow_low_prev = cv::Mat::zeros(flow.rows, flow.cols, CV_8U);
        flow_high_prev = cv::Mat::zeros(flow.rows, flow.cols, CV_8U);
        flow_hist = cv::Mat::zeros(flow.rows, flow.cols, CV_32FC1);
    }

    std::vector<cv::Mat> xy(2);
    cv::split(flow, xy);
    cv::cartToPolar(xy[0], xy[1], magnitude, angle, true);
    //
    // Compute the low speed mask
    // Shadow these variables:
    float adj_flow_threshold_low = flow_threshold_low * since_last_capture * 30.0;
    float adj_flow_threshold_high = flow_threshold_high * since_last_capture * 30.0;
    flow_low = magnitude > adj_flow_threshold_low; // & magnitude < adj_flow_threshold_high;
    cv::dilate(flow_low, flow_low, open_kernel);
    cv::erode(flow_low, flow_low, open_kernel);
    cv::erode(flow_low, flow_low, open_kernel);
    //
    // Compute the high speed mask
    flow_high = magnitude > adj_flow_threshold_high; // & flow_low_prev > 0;
    cv::erode(flow_high, flow_high, open_kernel);
    cv::dilate(flow_high, flow_high, open_kernel);
    flow_behind = flow_low_prev & (255 - flow_low);
    flow_new = flow_high & ( 255 - flow_high_prev);
    // Update history image
    cv::add(flow_hist, flow_behind, flow_hist, cv::noArray(), CV_32F);
    flow_hist *= 0.9;
    // Contourfinders
    contourfinder_low.findContours(flow_low);
    contourfinder_high.findContours(flow_high);
}



void FlowCam::computeRoi()
{
    float ratio = (float)screen_width / screen_height;
    int w = std::min(capture_width, static_cast<int>(round(capture_height * ratio)));
    int h = std::min(capture_height, static_cast<int>(round(capture_width / ratio)));
    w /= zoom;
    h /= zoom;
    capture_roi = cv::Rect((capture_width - w) / 2, (capture_height - h) / 2, w, h);
    flow_width = ceil(w / pow(2.0, pyrdown_steps));
    flow_height = ceil(h / pow(2.0, pyrdown_steps));
    reset();
}


void FlowCam::applyLUT()
{
    if (LUTloaded) {
        for(int y = 0; y < frame_screen_im.getHeight(); y++) {
            for(int x = 0; x < frame_screen_im.getWidth(); x++) {
                ofColor color = frame_screen_im.getColor(x, y);
                int lutPos [3];

                for (int m = 0; m < 3; m++) {
                    lutPos[m] = color[m] / 8;

                    if (lutPos[m] == 31) {
                        lutPos[m] = 30;
                    }
                }

                ofVec3f start = lut[lutPos[0]][lutPos[1]][lutPos[2]];
                ofVec3f end = lut[lutPos[0] + 1][lutPos[1] + 1][lutPos[2] + 1];

                for (int k = 0; k < 3; k++) {
                    float amount = (color[k] % 8) / 8.0f;
                    color[k] = (start[k] + amount * (end[k] - start[k])) * 255;
                }

                frame_screen_im.setColor(x, y, color);
            }
        }

        frame_screen_im.update();
    }
}
