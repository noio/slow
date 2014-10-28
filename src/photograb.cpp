#include "photograb.h"

void Photograb::setup(std::string path)
{
    border_im.loadImage("assets/photograb_border.png");
    refer_im.loadImage("assets/photograb_refer.png");
    font.loadFont("assets/squadaone.ttf", grab_width / 4);
    save_path = path;
    lut.load("LUTs/X_Pro_II.cube");
}

void Photograb::update(double delta_t)
{
    state_time += delta_t;
    
}

void Photograb::draw()
{
    switch (state) {
        case COUNTDOWN:
            ofEnableAlphaBlending();
            border_im.draw(topleft, grab_width, grab_height);
            ofDisableAlphaBlending();
            if (state_time > grab_delay)
            {
                capture();
                switchState(FADE);
            }
            ofSetColor(ofColor::white);
            font.drawString(ofToString((int)(grab_delay - state_time) + 1), topleft.x + 20, topleft.y + 20 + font.getSize());
            if (state_time > grab_delay - flash_time)
            {
                ofSetColor(255, 255, 255);
                ofFill();
                ofRect(0, 0, ofGetWidth(), ofGetHeight());
            }
//            ofSetColor(ofColor::white, state_time / grab_delay);
//            ofRect(topleft, grab_width, grab_height);
            break;
            
        case FADE:
            refer_im.draw(0,0, ofGetWidth(), ofGetHeight());
            ofEnableAlphaBlending();
            ofSetColor(255, 255, 255, 255 * (1.0f - (state_time / fade_time)));
            ofFill();
            ofRect(0, 0, ofGetWidth(), ofGetHeight());
            ofDisableAlphaBlending();
            screen.draw((ofGetWidth() - grab_width) / 2, (ofGetHeight() - grab_height) / 2, grab_width, grab_height);
            
            if (state_time > fade_time)
            {
                switchState(IDLE);
            }
            
        default:
            break;
    }
}

void Photograb::switchState(STATE new_state)
{
    state = new_state;
    state_time = 0;
}


void Photograb::grab(ofPoint position)
{
    // Set all parameters/positions correctly
    grab_height = grab_width * 1.2;
    topleft = position - ofPoint(grab_width / 2, grab_height / 2);
    topleft.x = ofClamp(floor(topleft.x), 0, ofGetWidth() - grab_width);
    topleft.y = ofClamp(floor(topleft.y), 0, ofGetHeight() - grab_height);
    
    switchState(COUNTDOWN);
}

void Photograb::capture()
{
    screen.grabScreen(topleft.x, topleft.y, grab_width, grab_height);
    lut.applyLUT(screen.getPixelsRef());
    std::string file_path = save_path + "/slow_artis_" + ofGetTimestampString("%Y%m%d-%H%M") + ".png";
    screen.saveImage(file_path);
}

