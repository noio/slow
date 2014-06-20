
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
    
    void bumpActivity();
    
private:
    void updateAmbientSound(double delta_t);
    ofSoundPlayer* playRandomSound(vector<ofSoundPlayer>& sounds);
    void loadSounds();
    void loadSoundPlayers(const vector<string>& filenames, vector<ofSoundPlayer>& sounds);

    ofSoundPlayer grab_sound, face_sound, siren_low_sound, siren_high_sound;
    vector<ofSoundPlayer> ambient_sounds;
    vector<ofSoundPlayer> scared_sounds;
    ofSoundPlayer* current_ambient_sound;
    
    bool sounds_on = true;
    
    double time_to_next_ambient = 0.0;
    float last_activity = 0.0;
    bool active = false;
    
    float ambient_cooldown_active_min = 10.0;
    float ambient_cooldown_active_max = 15.0;
    float ambient_cooldown_inactive_min = 15.0;
    float ambient_cooldown_inactive_max = 30.0;
    
};

#endif /* defined(__slow__ambientplayer__) */
