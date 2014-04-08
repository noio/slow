//
//  Particle.cpp
//  slow
//
//  Created by Thomas van den Berg on 08/04/14.
//
//

#include "Particle.h"

Particle::Particle(){
    alive = false;
}

void Particle::setup(){
    time = 0;
}

void Particle::update( float dt , const vector<double>& u, const vector<double>& v){
    if (alive){
        
    }
}

void Particle::draw(){
    ofColor(255,0,255);
    ofCircle(pos, 4);
}