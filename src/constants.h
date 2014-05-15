//
//  constants.h
//  Slow
//
//  Created by Thomas van den Berg on 27/01/2014.
//
//

#ifndef SLOW_CONSTANTS_H_
#define SLOW_CONSTANTS_H_

//const int kCaptureWidth = 640;
//const int kCaptureHeight = 360;
const int kCaptureWidth = 1280;
const int kCaptureHeight = 720;

const int kScreenWidth = 1080;
const int kScreenHeight = 480;
const cv::Size kScreenSize(kScreenWidth, kScreenHeight);

const float kGameWidth = kScreenWidth;
const float kGameHeight = kScreenHeight;
const cv::Size2f kGameSize(kGameWidth, kGameHeight);

const double kGameSizePadding = 10.0;

const double kPhysicsScale = 30.0;

const double kRatio = (float)kScreenWidth / (float)kScreenHeight;
const int kFrameWidth = std::min(kCaptureWidth, static_cast<int>(kCaptureHeight * kRatio));
const int kFrameHeight = std::min(kCaptureHeight, static_cast<int>(kCaptureWidth / kRatio));
const cv::Rect kCaptureROI = cv::Rect((kCaptureWidth - kFrameWidth) / 2, (kCaptureHeight - kFrameHeight) / 2, kFrameWidth, kFrameHeight);

const float kScaleFrameToScreen = (float)kScreenWidth / kFrameWidth;

const int kFlowWidth = kFrameWidth / 4;
const int kFlowHeight = kFrameHeight / 4;
const ofPoint kFlowSize(kFlowWidth, kFlowHeight, 1.0);

const double kFrameRate = 40.0;

//const int kScreenWidth = 640;
//const int kScreenHeight = 480;

const int kFluidWidth = kScreenWidth / 6;
const int kFluidHeight = kScreenHeight / 6;
const ofPoint kFluidSize(kFluidWidth, kFluidHeight, 1.0);

const int kMaxParticles = 1000;


// Other
const ofPoint kLabelOffset = ofPoint(3, 13);

#endif
