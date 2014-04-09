//
//  Particle.cpp
//  slow
//
//  Created by Thomas van den Berg on 08/04/14.
//
//

#include "particle.h"

#include "constants.h"

Particle::Particle(){
    alive = false;
}

void Particle::setup(){
    time = 0;
    alive = true;
}

void Particle::update( float dt , FluidSolver fluid){
    if (alive){
        Point v = fluid.velocity_at(pos.x / kGameWidth, pos.y / kGameHeight);
        pos += vel;
    }
}

void Particle::draw(){
    ofColor(255,0,255);
    ofCircle(pos, 4);
}