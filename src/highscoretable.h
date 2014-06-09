
#ifndef SLOW_HIGHSCORE_TABLE_H_
#define SLOW_HIGHSCORE_TABLE_H_

#include "framerecord.h"

#include "ofMain.h"

#include <iostream>
#include <vector>
#include <algorithm>

using std::sort;
using std::vector;

typedef struct Highscore {
    double time;
    ofPtr<FrameRecord> recording;
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

    void setup(float w);
    void update(double delta_t);
    void draw();
    
    void add(double time, ofPtr<FrameRecord> recording);
private:
    float width;
    vector<Highscore> scores;
    
};

#endif /* defined(SLOW_HIGHSCORE_TABLE_H_) */
