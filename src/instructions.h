
#ifndef SLOW_INSTRUCTIONS_H_
#define SLOW_INSTRUCTIONS_H_

#include "squid.h"
#include "ofMain.h"
#include "ofxPlaylist.h"
#include <iostream>
#include <vector>

class Instructions {
public:
    Instructions(){};
    Instructions(const Instructions&) = delete;            // no copy
    Instructions& operator=(const Instructions&) = delete; // no assign

    void setup(Squid* squid);
    void update(double delta_t);
    void draw();
    
    void play();
    
    float instruction_cooldown = 300;
    
    
private:
    Squid* squid;
    
    ofxPlaylist playlist;
    vector<ofImage> instructions_ims;
    vector<float> instructions_alpha;

    float draw_height;
    ofPoint draw_pos;
    
    float time_last_instructions = 0.0;
    
};


#endif /* defined(SLOW_INSTRUCTIONS_H_) */
