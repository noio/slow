
#include "highscoretable.h"

void HighscoreTable::setup(float w){
    width = w;
}

void HighscoreTable::add(double time, ofPtr<FrameRecord> recording){
    Highscore h = {time, recording};
    scores.push_back(h);
    sort(scores.begin(), scores.end(), compareScore);
}

void HighscoreTable::update(double delta_t){
    for (int i = 0; i < scores.size(); i ++){
        scores[i].recording->update(delta_t);
    }
}

void HighscoreTable::draw(){
    for (int i = 0; i < scores.size(); i ++){
        ofEnableAlphaBlending();
        ofSetColor(ofColor::white);
        scores[i].recording->draw(0, i * width, width, width);
        ofDrawBitmapStringHighlight(ofToString(scores[i].time, 0) + " seconds", ofPoint(10, i*width + 10));
    }
}