
#ifndef SLOW_HIGHSCORE_TABLE_H_
#define SLOW_HIGHSCORE_TABLE_H_

#include "framerecord.h"
#include "motionvisualizer.h"
#include "ofMain.h"

#include <iostream>
#include <vector>
#include <algorithm>

using std::sort;
using std::vector;

typedef struct Highscore {
    double time;
    unsigned int serial;
    ofPtr<FrameRecord> recording;
    bool operator == (const Highscore &i) const {return serial == i.serial;}
}
Highscore;

inline bool compareScore(const Highscore& a, const Highscore& b){
    return a.time > b.time;
}

class HighscoreTable {
public:
    HighscoreTable(){};
    HighscoreTable(const HighscoreTable&) = delete;            // no copy
    HighscoreTable& operator=(const HighscoreTable&) = delete; // no assign

    void setup(float w, MotionVisualizer* visualizer);
    void update(double delta_t);
    void draw();
    
    void add(double time, ofPtr<FrameRecord> recording);
    
    int max_scores = 4;
private:
    MotionVisualizer* visualizer;
    ofTrueTypeFont squada;
    float width, row_height;
    vector<Highscore> scores;
    unsigned int serial = 0;
    
};

#endif /* defined(SLOW_HIGHSCORE_TABLE_H_) */
