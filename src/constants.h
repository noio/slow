//
//  constants.h
//  Slow
//
//  Created by Thomas van den Berg on 27/01/2014.
//
//

#ifndef SLOW_CONSTANTS_H_
#define SLOW_CONSTANTS_H_

const int kCaptureWidth = 640;
const int kCaptureHeight = 360;
//const int kCaptureWidth = 1280;
//const int kCaptureHeight = 720;

const int kScreenWidth = 1080;
const int kScreenHeight = 480;
const ofPoint kScreenSize(kScreenWidth, kScreenHeight, 1.0);

const float kGameWidth = kScreenWidth;
const float kGameHeight = kScreenHeight;
const ofPoint kGameSize(kGameWidth, kGameHeight, 1.0);

const float kGameSizePadding = 20.0;

const float kRatio = (float)kScreenWidth / (float)kScreenHeight;
const int kROIWidth = std::min(kCaptureWidth, static_cast<int>(kCaptureHeight* kRatio));
const int kROIHeight = std::min(kCaptureHeight, static_cast<int>(kCaptureWidth / kRatio));
const cv::Rect kCaptureROI = cv::Rect((kCaptureWidth - kROIWidth) / 2, (kCaptureHeight - kROIHeight) / 2, kROIWidth, kROIHeight);

const int kFlowWidth = kROIWidth / 4;
const int kFlowHeight = kROIHeight / 4;
const ofPoint kFlowSize(kFlowWidth, kFlowHeight, 1.0);

const float kFrameRate = 40.0;


//const int kScreenWidth = 640;
//const int kScreenHeight = 480;

const int kFluidWidth = kScreenWidth / 6;
const int kFluidHeight = kScreenHeight / 6;
const ofPoint kFluidSize(kFluidWidth, kFluidHeight, 1.0);

const int kMaxParticles = 1000;

//GAMEPLAY
const float kFlowLowThreshold = 0.1f;
const float kFlowHighThreshold = 0.3f;

const float kFlowErosionSize = 5.0f;

#endif
