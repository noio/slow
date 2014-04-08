//
//  constants.h
//  Slow
//
//  Created by Thomas van den Berg on 27/01/2014.
//
//

#ifndef SLOW_CONSTANTS_H
#define SLOW_CONSTANTS_H

const float RAD2DEG = 180 / 3.141593f;

const int kCaptureWidth = 640;
const int kCaptureHeight = 360;
//const int kCaptureWidth = 1280;
//const int kCaptureHeight = 720;

const int kScreenWidth = 1080;
const int kScreenHeight = 480;

//const int kScreenWidth = 640;
//const int kScreenHeight = 480;

const int kFluidWidth = 160;
const int kFluidHeight = 90;

const float kGameWidth = 360.0f;
const float kGameHeight = 160.0f;
const float kGameWorldPadding = 20.0f;

//GAMEPLAY
const int kHistoryInterval = 2;
const int kHistoryMaxSize = 30;

const int kFlowFrameSkip = 2;

const int kFlowInputMaxWidth = 160;

const float kFlowLowThreshold = 0.1f;
const float kFlowHighThreshold = 0.3f;

const float kFlowErosionSize = 3.0f;
const float kFlowSensitivityDecay = 0.95f;
const float kFlowSensitivityMultiplier = 0.0005f;

const float kRepelForceMultiplier = 10.f;
const float kRepelForceVerticalBias = -1.0f;
const float kRepelMaxDistance = 40.f;

#endif
