
#ifndef SLOW_INSTRUCTIONS_H_
#define SLOW_INSTRUCTIONS_H_

#include "squid.h"
#include "ofMain.h"
#include "ofxPlaylist.h"
#include <iostream>

class Instructions {
public:
    void setup(Squid* squid);
    void update(double delta_t);
    void draw();
    
private:
    ofxPlaylist playlist;
    ofImage instructions_1_im, instructions_2_im, instructions_3_im;
    
};


#endif /* defined(SLOW_INSTRUCTIONS_H_) */
