
#ifndef SLOW_UTILITIES_H_
#define SLOW_UTILITIES_H_

#include "ofMain.h"
#include "ofxCv.h"

inline ofVec3f operator/(const ofVec3f& point, const cv::Size& size )
{
    return ofVec3f( point.x / size.width, point.y / size.height, point.z );
}

inline ofVec3f operator*(const ofVec3f& point, const cv::Size& size )
{
    return ofVec3f( point.x * size.width, point.y * size.height, point.z );
}

#endif
