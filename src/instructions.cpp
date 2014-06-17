
#include "instructions.h"

using namespace Playlist;

void Instructions::setup(Squid* in_squid)
{
    squid = in_squid;
    instructions_ims.resize(7);
    instructions_alpha.resize(7, 0.0);
    instructions_ims[0].loadImage("assets/instructions_1.png");
    instructions_ims[1].loadImage("assets/instructions_2.png");
    instructions_ims[2].loadImage("assets/instructions_3.png");
    instructions_ims[3].loadImage("assets/instructions_4.png");
    instructions_ims[4].loadImage("assets/instructions_5.png");
    instructions_ims[5].loadImage("assets/instructions_6.png");
    instructions_ims[6].loadImage("assets/instructions_7.png");
    draw_height = instructions_ims[0].height * (ofGetWidth() / (float)instructions_ims[0].width);
    draw_pos = ofPoint(0, (ofGetHeight() - draw_height) / 2);
}

void Instructions::update(double delta_t)
{
    playlist.update();
    float cur = ofGetElapsedTimef();
    if ((cur - squid->getTimeLastFace() > instruction_cooldown) &&
        (cur - time_last_instructions > instruction_cooldown)){
        play();
    }
}

void Instructions::draw()
{
    for (int i = 0; i < instructions_ims.size(); i++) {
        if (instructions_alpha[i] > 0.1) {
            ofEnableAlphaBlending();
            ofSetColor(255, 255, 255, 255);
            instructions_ims[i].draw(draw_pos, ofGetWidth(), draw_height);
            ofSetColor(255, 200, 255, 255 * (1.0 - instructions_alpha[i]));
            ofRect(0, 0, ofGetWidth(), ofGetHeight());
            ofDisableAlphaBlending();
        }
    }
}

void Instructions::play()
{
    ofLogVerbose("Instructions") << "starting script";
    time_last_instructions = ofGetElapsedTimef();
    squid->stayAtPoint(ofPoint(ofGetWidth() * 0.81, ofGetHeight() * 0.40), 15 + 4);
    ofColor yellow(255,255,0, 255);
    ofColor black(0,0,0,255);
    squid->switchColorsTemp(yellow, black, black, yellow, black, 0.2f, 5.0f);

    // First three screens contain instructions, show slowly
    for (int i = 0; i < 3; i++) {
        playlist.addToKeyFrame(Action::tween(200.0f, &instructions_alpha[i], 1.0));
        playlist.addKeyFrame(Action::pause(4800.0f));
        playlist.addKeyFrame(Action::tween(200.0f, &instructions_alpha[i], 0.0));
    }

    // Rest is countdown
    for (int i = 3; i < instructions_ims.size(); i ++) {
        playlist.addToKeyFrame(Action::tween(200.0f, &instructions_alpha[i], 1.0));
        playlist.addKeyFrame(Action::pause(800.0f));
        playlist.addKeyFrame(Action::tween(200.0f, &instructions_alpha[i], 0.0));
    }
}