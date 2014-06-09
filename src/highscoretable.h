
#ifndef SLOW_HIGHSCORE_TABLE_H_
#define SLOW_HIGHSCORE_TABLE_H_

#include <iostream>

class HighscoreTable {
public:
    HighscoreTable(){};
    HighscoreTable(const HighscoreTable&) = delete;            // no copy
    HighscoreTable& operator=(const HighscoreTable&) = delete; // no assign

    void setup();
private:
};

#endif /* defined(SLOW_HIGHSCORE_TABLE_H_) */
