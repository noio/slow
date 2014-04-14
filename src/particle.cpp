//
//  Particle.cpp
//  slow
//
//  Created by Thomas van den Berg on 08/04/14.
//
//

#include "particle.h"

#include "constants.h"

using std::cout;
using std::endl;

Particle::Particle(){
    alive = false;
}

void Particle::setup( float x, float y){
    int r = ofRandom(3);
    if (r == 0){
        color = ofColor(255,0,0);
    } else if (r == 1){
        color = ofColor(0,255,0);
    } else if (r == 2){
        color = ofColor(0,0,255);
    }
    age = 0;
    alive = true;
    pos.x = x;
    pos.y = y;
}

void Particle::update( float dt , FluidSolver* fluid){
    if (alive){
        Velocity v = fluid->velocity_at(pos.x / kGameWidth, pos.y / kGameHeight);
//        vel.x = 0.5 * (vel.x + v.u);
//        vel.y = 0.5 * (vel.y + v.v);
        vel.x = v.u * kGameWidth;
        vel.y = v.v * kGameHeight;
        pos += vel * dt;
//        vel *= 0.9;
        
        if (pos.x < 0) vel.x = abs(vel.x);
        if (pos.x > kGameWidth) vel.x = -abs(vel.x);
        if (pos.y < 0) vel.y = abs(vel.y);
        if (pos.y > kGameHeight) vel.y = -abs(vel.y);
        
        age += dt;
        if (age > 10.0){
            alive = false;
        }
    }
}

void Particle::draw(){
    if (alive){
        ofSetColor(color);
        ofCircle(pos, 6);
    }
}