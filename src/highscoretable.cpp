
#include "highscoretable.h"

void HighscoreTable::setup(float w, MotionVisualizer* in_visualizer)
{
    width = w;
    row_height = w;
    visualizer = in_visualizer;
    squada.loadFont("assets/squadaone.ttf", 20);
}

void HighscoreTable::add(double time, ofPtr<FrameRecord> recording)
{
    Highscore h = {time, serial++, recording};
    scores.push_back(h);
    sort(scores.begin(), scores.end(), compareScore);
    vector<Highscore>::iterator it = std::find(scores.begin(), scores.end(), h);
    int pos = std::distance(scores.begin(), it);
    ofLogVerbose("HighscoreTable") << "new score at pos: " << pos << endl;

    if (pos < max_scores) {
        visualizer->sparkle(ofPoint(width / 2, (pos + 0.5) * row_height), width / 2);
    }

    if (scores.size() > max_scores) {
        scores.resize(max_scores);
    }
}

void HighscoreTable::update(double delta_t)
{
    for (int i = 0; i < scores.size(); i ++) {
        scores[i].recording->update(delta_t);
    }
}

void HighscoreTable::draw()
{
    for (int i = 0; i < scores.size(); i ++) {
        ofEnableAlphaBlending();
        ofSetColor(ofColor::white);
        scores[i].recording->draw(0, i * row_height, width, row_height);
        string scoretxt = ofToString(round(scores[i].time) * 10, 0);
        ofRectangle bounds = squada.getStringBoundingBox(scoretxt, 0, 0);
        squada.drawString(scoretxt, (width - bounds.width) / 2, i * row_height + (row_height - bounds.height / 2));
//        ofDrawBitmapStringHighlight(ofToString(scores[i].time, 0) + " seconds", ofPoint(10, i * width + 10));
    }
}