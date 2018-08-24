#pragma once

#include "ofMain.h"
#include "ofxImGui.h"

#include "ofxXmlSettings.h"
#include "YAMPE/Particle.h"
#include "YAMPE/Particle/ForceGeneratorRegistry.h"
#include "YAMPE/Particle/ContactRegistry.h"
#include "YAMPE/Particle/ContactGenerators.h"
#include "YAMPE\Particle\Constraints.h"



//Author James Power 20067779


class ofApp : public ofBaseApp {
    
public:
    void setup();
    void update();
    void draw();
    
    void keyPressed(int key);
    void keyReleased(int key);
    void mouseMoved(int x, int y );
    void mouseDragged(int x, int y, int button);
    void mousePressed(int x, int y, int button);
    void mouseReleased(int x, int y, int button);
    void mouseEntered(int x, int y);
    void mouseExited(int x, int y);
    void windowResized(int w, int h);
    void dragEvent(ofDragInfo dragInfo);
    void gotMessage(ofMessage msg);
		
    // simple 3D world with ground and axes
    const float RANGE = 16;
    ofEasyCam easyCam;
    float cameraHeightRatio = 0.02f;
    ofPoint easyCamTarget = ofPoint(0,5,0);
    void cameraHeightRatioChanged(float & cameraHeightRatio);

    ofPlanePrimitive ground;
    
    ofxImGui::Gui gui;                           // single entery to ImGUI subsystem
    ofRectangle mainWindowRectangle;        // used to ignore mouse drags for camera
    ofRectangle loggingWindowRectangle;     // used to ignore mouse drags for camera
    void drawAppMenuBar();
    void drawMainWindow();
    void drawLoggingWindow();
    
    // simimulation (generic)
    void reset();
    void quit();
    float t = 0.0f;
    bool isRunning = true;
    
    ofParameter<bool> isAxisVisible = true;
    ofParameter<bool> isXGridVisible = false;
    ofParameter<bool> isYGridVisible = true;;
    ofParameter<bool> isZGridVisible = false;;
    ofParameter<bool> isGroundVisible = true;
    ofParameter<bool> isFullScreen;
    ofParameter<std::string> position;

    // TODO - simimulation (specific stuff)

	int numBalls = 3;	//the amount of balls to be generated
	YAMPE::ParticleRegistry particles;	//particle registry from YAMPE/Particle

	int numBallsPerturbed = 1;	//num balls at an angle
	float perturbAngle = 45.0f;			//angle balls  will bee pulled back at
	float gap = 0.0f;			//gap between balls

	float xStartPosition = 0.0f;	//used to centre the balls on screen

	YAMPE::P::ForceGeneratorRegistry  forceGeneratorRegistry;
	YAMPE::P::GravityForceGenerator::Ref gravity;

	ofVec3f gravityVector;

	YAMPE::P::ContactRegistry::Ref contacts;	//contacts registry
	YAMPE::P::ParticleParticleContactGenerator ppContactGenerator;

	vector<YAMPE::P::EqualityAnchoredConstraint::Ref> anchorConstraints;

	//constants
	const int MAX_BALLS = 20;
	const int MIN_BALLS = 1;
	const float BALL_RADIUS = 0.5f;
	const float BALL_HEIGHT = 5.0f;
	const float ANCHOR_HEIGHT = 10.0f;
	const float ANCHOR_LENGTH = 5.0f;
	const float MAX_ANGLE = 90.0f;
	const float MIN_ANGLE = 0.0f;






private:

    // or here

};
