#pragma once

#include "ofxLUT.h"
#include "ofMain.h"
#include <string>

class Photograb {

    enum STATE {
        IDLE,
        COUNTDOWN,
        FADE
    };
    
public:
    
    void setup(std::string path);
    void update(double delta_t);
    void draw();
    
    void grab(ofPoint position);
    
    float grab_delay = 3.2;
    float flash_time = 0.1;
    float fade_time = 3;
    int grab_width = 200;
    bool do_save = false;
    
private:
    
    void capture();
    void switchState(STATE new_state);
    
    
    STATE state;
    double state_time;
    
    ofImage border_im, refer_im;
    
    int grab_height;
    ofPoint topleft;
    ofTrueTypeFont font;
    
    std::string save_path;
    ofImage screen;
    ofxLUT lut;
};