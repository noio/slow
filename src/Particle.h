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

using std::vector;

//Particle class
class Particle {
public:
	Particle();
	void setup();
	void update( float dt , const vector<double>& u, const vector<double>& v);
	void draw();
    
	ofPoint pos;
	ofPoint vel;
	float time;
	bool alive;
};

#endif /* defined(__slow__Particle__) */
