
#include "ambientplayer.h"

void AmbientPlayer::setup(){
    loadSounds();
}

void update(double delta_t){
    float cur = ofGetElapsedTimef();
//    if (cur - time_last_ambient > ambient_cooldown) {
//        
//    }
    
}

void AmbientPlayer::playScared(){
    scared_sounds[ofRandomuf() * scared_sounds.size()].play();
}

void AmbientPlayer::playGrab(){
    grab_sound.play();
}

void AmbientPlayer::stopGrab(){
    grab_sound.stop();
}

void AmbientPlayer::playFace(){
    face_sound.play();
}

void AmbientPlayer::setSoundsOn(bool in_sounds_on){
    sounds_on = in_sounds_on;
}

void AmbientPlayer::loadSounds(){
    grab_sound.loadSound("sound/fx/grab.mp3");
    face_sound.loadSound("sound/fx/pop.mp3");
    siren_high_sound.loadSound("sound/fx/slow-hoog.mp3");
    siren_low_sound.loadSound("sound/fx/slow_laag.mp3");
    
    vector<string> scared_sounds_filenames;
    scared_sounds_filenames.push_back("sound/notes/1 scharry.mp3");
    scared_sounds_filenames.push_back("sound/notes/2 scharry.mp3");
    scared_sounds_filenames.push_back("sound/notes/3 scharry.mp3");
    scared_sounds_filenames.push_back("sound/notes/4 scharry.mp3");
    scared_sounds_filenames.push_back("sound/notes/5 scharry.mp3");
    scared_sounds_filenames.push_back("sound/notes/6 scharry.mp3");
    scared_sounds_filenames.push_back("sound/notes/7 scharry.mp3");
    scared_sounds_filenames.push_back("sound/notes/8 scharry.mp3");
    scared_sounds_filenames.push_back("sound/notes/9 scharry.mp3");
    scared_sounds_filenames.push_back("sound/notes/10 scharry.mp3");
    loadSoundPlayers(scared_sounds_filenames, scared_sounds);
    
    vector<string> ambient_sounds_filenames;
    ambient_sounds_filenames.push_back("sound/notes/1 airy-low.mp3");
    ambient_sounds_filenames.push_back("sound/notes/1 ariry-hi.mp3");
    ambient_sounds_filenames.push_back("sound/notes/1 bell-hi.mp3");
    ambient_sounds_filenames.push_back("sound/notes/1 bell-low.mp3");
    ambient_sounds_filenames.push_back("sound/notes/2 airy-low.mp3");
    ambient_sounds_filenames.push_back("sound/notes/2 ariry-hi.mp3");
    ambient_sounds_filenames.push_back("sound/notes/2 bell-hi.mp3");
    ambient_sounds_filenames.push_back("sound/notes/2 bell-low.mp3");
    ambient_sounds_filenames.push_back("sound/notes/3 airy-low.mp3");
    ambient_sounds_filenames.push_back("sound/notes/3 ariry-hi.mp3");
    ambient_sounds_filenames.push_back("sound/notes/3 bell-hi.mp3");
    ambient_sounds_filenames.push_back("sound/notes/3 bell-low.mp3");
    ambient_sounds_filenames.push_back("sound/notes/4 airy-low.mp3");
    ambient_sounds_filenames.push_back("sound/notes/4 ariry-hi.mp3");
    ambient_sounds_filenames.push_back("sound/notes/4 bell-hi.mp3");
    ambient_sounds_filenames.push_back("sound/notes/4 bell-low.mp3");
    ambient_sounds_filenames.push_back("sound/notes/5 airy-low.mp3");
    ambient_sounds_filenames.push_back("sound/notes/5 ariry-hi.mp3");
    ambient_sounds_filenames.push_back("sound/notes/5 bell-hi.mp3");
    ambient_sounds_filenames.push_back("sound/notes/5 bell-low.mp3");
    ambient_sounds_filenames.push_back("sound/notes/6 airy-low.mp3");
    ambient_sounds_filenames.push_back("sound/notes/6 ariry-hi.mp3");
    ambient_sounds_filenames.push_back("sound/notes/6 bell-hi.mp3");
    ambient_sounds_filenames.push_back("sound/notes/6 bell-low.mp3");
    ambient_sounds_filenames.push_back("sound/notes/7 airy-low.mp3");
    ambient_sounds_filenames.push_back("sound/notes/7 ariry-hi.mp3");
    ambient_sounds_filenames.push_back("sound/notes/7 bell-hi.mp3");
    ambient_sounds_filenames.push_back("sound/notes/7 bell-low.mp3");
    ambient_sounds_filenames.push_back("sound/notes/8 airy-low.mp3");
    ambient_sounds_filenames.push_back("sound/notes/8 ariry-hi.mp3");
    ambient_sounds_filenames.push_back("sound/notes/8 bell-hi.mp3");
    ambient_sounds_filenames.push_back("sound/notes/8 bell-low.mp3");
    ambient_sounds_filenames.push_back("sound/notes/9 airy-low.mp3");
    ambient_sounds_filenames.push_back("sound/notes/9 ariry-hi.mp3");
    ambient_sounds_filenames.push_back("sound/notes/9 bell-hi.mp3");
    ambient_sounds_filenames.push_back("sound/notes/9 bell-low.mp3");
    ambient_sounds_filenames.push_back("sound/notes/10 airy-low.mp3");
    ambient_sounds_filenames.push_back("sound/notes/10 ariry-hi.mp3");
    ambient_sounds_filenames.push_back("sound/notes/10 bell-hi.mp3");
    ambient_sounds_filenames.push_back("sound/notes/10 bell-low.mp3");
    loadSoundPlayers(ambient_sounds_filenames, ambient_sounds);
}

void AmbientPlayer::loadSoundPlayers(const vector<string>& filenames, vector<ofSoundPlayer>& sounds){
    sounds.resize(filenames.size());
    for (int i = 0; i < filenames.size(); i ++){
        sounds[i].loadSound(filenames[i]);
    }
}




