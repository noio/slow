
#ifndef SLOW_AMBIENTPLAYER_H_
#define SLOW_AMBIENTPLAYER_H_

#include "ofMain.h"

#include <string>
#include <iostream>

class AmbientPlayer {
public:
    void setup();
    void update(double delta_t);
    
    void setSoundsOn(bool in_sounds_on);
    
    void playGrab();
    void stopGrab();
    void playFace();
    void playScared();
    
private:
    void loadSounds();
    void loadSoundPlayers(const vector<string>& filenames, vector<ofSoundPlayer>& sounds);

    ofSoundPlayer grab_sound, face_sound, siren_low_sound, siren_high_sound;
    vector<ofSoundPlayer> ambient_sounds;
    vector<ofSoundPlayer> scared_sounds;
    ofSoundPlayer* current_ambient_sound;
    
    bool sounds_on = true;
    
    float time_next_ambient = 0.0;
    float ambient_cooldown_min = 10.0;
    float ambient_cooldown_max = 20.0;
    
};

#endif /* defined(__slow__ambientplayer__) */
