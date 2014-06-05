
#ifndef SLOW_UTILITIES_H_
#define SLOW_UTILITIES_H_

#include "ofMain.h"
#include "ofxCv.h"
#include "constants.h"
#include "Box2D/Box2D.h"

inline ofVec3f operator/(const ofVec3f& point, const cv::Size& size )
{
    return ofVec3f( point.x / size.width, point.y / size.height, point.z );
}

inline ofVec3f operator*(const ofVec3f& point, const cv::Size& size )
{
    return ofVec3f( point.x * size.width, point.y * size.height, point.z );
}

inline ofPoint b2ToOf (const b2Vec2 point){
    return ofPoint(point.x * kPhysicsScale, point.y * kPhysicsScale);
}

inline b2Vec2 ofToB2 (const ofPoint point){
    return b2Vec2(point.x / kPhysicsScale, point.y / kPhysicsScale);
}

inline b2AABB ofToB2 (const ofRectangle rect){
    b2AABB output;
    output.lowerBound = b2Vec2(rect.x / kPhysicsScale, rect.y / kPhysicsScale);
    output.upperBound = b2Vec2((rect.x + rect.width) / kPhysicsScale, (rect.y + rect.height) / kPhysicsScale);
    return output;
}

inline void printMatrixInfo(const cv::Mat& mat) {
    cv::string r;
    
    uchar depth = mat.type() & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (mat.type() >> CV_CN_SHIFT);
    
    switch ( depth ) {
        case CV_8U:  r = "8U"; break;
        case CV_8S:  r = "8S"; break;
        case CV_16U: r = "16U"; break;
        case CV_16S: r = "16S"; break;
        case CV_32S: r = "32S"; break;
        case CV_32F: r = "32F"; break;
        case CV_64F: r = "64F"; break;
        default:     r = "User"; break;
    }
    
    r += "C";
    r += (chans+'0');
    
    printf("Matrix: %s %dx%d \n", r.c_str(), mat.cols, mat.rows );
}

inline ofPoint getScreenSize(){
    return ofPoint(ofGetWidth(), ofGetHeight());
}

#endif
