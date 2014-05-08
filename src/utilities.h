
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

#endif
