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

const float kFrameRate = 40.0;

const float kGameWidth = kScreenWidth;
const float kGameHeight = kScreenHeight;
const float kGameWorldPadding = 20.0f;

//const int kScreenWidth = 640;
//const int kScreenHeight = 480;

const int kFluidWidth = kScreenWidth / 6;
const int kFluidHeight = kScreenHeight / 6;

const int kMaxParticles = 1000;

//GAMEPLAY
const float kFlowLowThreshold = 0.1f;
const float kFlowHighThreshold = 0.3f;

const float kFlowErosionSize = 5.0f;

#endif
