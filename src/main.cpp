#include "ofMain.h"
#include "ofApp.h"

#include "constants.h"

//========================================================================
int main( )
{
    ofSetupOpenGL(896, 288, OF_WINDOW);			// <-------- setup the GL context
    // this kicks off the running of my app
    // can be OF_WINDOW or OF_FULLSCREEN
    // pass in width and height too:
    ofRunApp(new ofApp());
}
