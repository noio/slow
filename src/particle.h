//
//  Particle.h
//  slow
//
//  Created by Thomas van den Berg on 08/04/14.
//
//

#ifndef SLOW_PARTICLE_H_
#define SLOW_PARTICLE_H_

#include <iostream>
#include <vector>

#include "ofMain.h"

#include "fluid.h"

using std::vector;

//Particle class
class Particle {
public:
	Particle();
	void setup(float x, float y);
	void update(float dt , FluidSolver* fluid);
	void draw();
    
	ofPoint pos;
	ofPoint vel;
    ofColor color;
    ofRectangle bounds;
	float age;
	bool alive;
};

#endif /* defined(__slow__Particle__) */
