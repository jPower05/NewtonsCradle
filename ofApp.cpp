#include "ofApp.h"
using namespace YAMPE; using namespace P;



//Author James Power 20067779




//--------------------------------------------------------------
void ofApp::setup() {
    
    ofSetLogLevel(OF_LOG_VERBOSE);
    
    // repatable randomness
    ofSeedRandom(10);
    
    // simulation specific stuff goes here

    gui.setup();
    ImGui::GetIO().MouseDrawCursor = false;

    // load parameters from file
    // loadFromFile("settings.xml");
    
    // instantiate the ground
    ground.set(RANGE, RANGE);
    ground.rotate(90, 1,0,0);
    
    // lift camera to 'eye' level
    easyCam.setDistance(RANGE);
    float d = easyCam.getDistance();
    easyCam.setPosition(ofVec3f(0, cameraHeightRatio*d, d*sqrt(1.0f-cameraHeightRatio*cameraHeightRatio))+easyCamTarget);
    easyCam.setTarget(easyCamTarget);

    // TODO - simulation specific stuff goes here
	gravityVector = ofVec3f(0.0f, -9.8f, 0.0f);

	gravity = GravityForceGenerator::Ref(new GravityForceGenerator(gravityVector, "gravity"));
	contacts = ContactRegistry::Ref(new ContactRegistry);
    // finally start everything off by resetting the simulation
    reset();
    
}

void ofApp::reset() {

    t = 0.0f;
    
    // TODO - simulation specific stuff goes here

	//clear all registrys

	particles.clear();	//clear particles registry
	forceGeneratorRegistry.clear(); //clear force generator registry
	ppContactGenerator.particles.clear();	//clear particle particle 
	anchorConstraints.clear();	//clear anchor constraints

	float distanceBetweenBalls = BALL_RADIUS * 2 + gap;
	xStartPosition = -(numBalls * (distanceBetweenBalls)) / 2;
	float xPos = xStartPosition;
	
	//create ball particles

	for (int k = 0; k < numBalls; ++k) {
		//generate particle with random POSITION AND ADD TO REGISTRY
		Particle::Ref p = Particle::Ref(new Particle());
		//set position	of anchor
		ofVec3f anchor;
		anchor.x = xPos;
		anchor.y = ANCHOR_HEIGHT;
		anchor.z = 0.0f;

		//set position of ball
		ofVec3f ballPosition;
		
		//if the number of balls at an angle is greater than the current k ball count. 
		//ie: still more balls that need angle set

		if (k < numBallsPerturbed){
			float angle = 90.0f - perturbAngle;
			ballPosition.x -= (ANCHOR_LENGTH * ofRadToDeg(cosf(ofDegToRad(angle))));
			ballPosition.y -= (ANCHOR_LENGTH * ofRadToDeg(sinf(ofDegToRad(angle))));
			ballPosition.z = 0.0f;
		}
		else {	//don't need to put ball at angle
			ballPosition.x = xPos;
			ballPosition.y -= ANCHOR_LENGTH;
		}
		
		p->setPosition(ballPosition);	
		p->setRadius(BALL_RADIUS);
		p->setBodyColor({ ofRandom(0, 255), ofRandom(0, 255), ofRandom(0, 255) });
		
		//create equality anchor constraint between the particle ball and the anchor point
		//fixed length of ANCHOR_LENGTH
		EqualityAnchoredConstraint::Ref constraint = EqualityAnchoredConstraint::Ref(new EqualityAnchoredConstraint(p, anchor, ANCHOR_LENGTH));

		anchorConstraints.push_back(constraint);
		//add to registry
		particles.push_back(p);
		//add gravity for each particle
		forceGeneratorRegistry.add(p, gravity);
		
		//add particle to particle contacts
		ppContactGenerator.particles.push_back(p);

		//increment xPos
		xPos += distanceBetweenBalls;
	}

	

    
}

void ofApp::update() {

    float dt = ofClamp(ofGetLastFrameTime(), 0.0, 0.02);
    if (!isRunning || dt<=0.0f) return;
    t += dt;



    // TODO - simulation specific stuff goes here
	forceGeneratorRegistry.applyForce(dt);

	// update all particles

	for (auto p : particles) (*p).integrate(dt);

	//update constraints 
	for (auto && c : anchorConstraints) c->generate(contacts);

	ppContactGenerator.generate(contacts);
	
	contacts->resolve(dt);
	contacts->clear();
}

void ofApp::draw() {
    
    ofEnableDepthTest();
    ofBackgroundGradient(ofColor(128), ofColor(0), OF_GRADIENT_BAR);
    
    ofPushStyle();
    easyCam.begin();
    
    ofDrawGrid(RANGE/(2*8), 8, false, isXGridVisible, isYGridVisible, isZGridVisible);
    
    if (isAxisVisible) ofDrawAxis(1);
    
    if (isGroundVisible) {
        ofPushStyle();
        ofSetHexColor(0xB87333);
        ground.draw();
        ofPopStyle();
    }
    
    // TODO - simulation specific stuff goes here

	//drawing newtons cradle
	
	float xPos = xStartPosition;
	float distanceBetweenBalls = BALL_RADIUS * 2 + gap;	//diameter of ball + gap between them

	for (auto p : particles) {
		ofSetColor({ ofRandom(0, 255), ofRandom(0, 255), ofRandom(0, 255) });	//strobe lights
		//ofSetColor(255, 0, 0);	//boring approach
		ofDrawLine(xPos, ANCHOR_HEIGHT, p->position.x, p->position.y);	//line that simulates equality anchor constraint
		//ofSetColor(255, 255, 0);	
		ofDrawSphere(xPos, ANCHOR_HEIGHT, 0.1f);	//anchor point
		p->draw();	//drawing particle (balls)
		xPos += distanceBetweenBalls;
	}
	//top line joining anchors visually
	ofDrawLine(-(particles.size() * distanceBetweenBalls), ANCHOR_HEIGHT, particles.size() * distanceBetweenBalls, ANCHOR_HEIGHT);



    
    easyCam.end();
    ofPopStyle();

    // draw gui elements
    gui.begin();
    drawAppMenuBar();
    drawMainWindow();
    drawLoggingWindow();
    gui.end();
    
}


void ofApp::drawAppMenuBar() {
    
    if (ImGui::BeginMainMenuBar()) {
        if (ImGui::BeginMenu("File")) {
            ImGui::Separator();
            if (ImGui::MenuItem("Quit", "")) quit();
            ImGui::EndMenu();
        }
        
        float d = easyCam.getDistance();
        
        if (ImGui::BeginMenu("View")) {
            if (ImGui::MenuItem(isAxisVisible?"Hide Unit Axis":"Show Unit Axis", "")) isAxisVisible = !isAxisVisible;
            if (ImGui::MenuItem(isGroundVisible?"Hide Ground":"Show Ground", "")) isGroundVisible = !isGroundVisible;
            if (ImGui::MenuItem(isXGridVisible?"Hide Grid (X)":"Show Grid (X)", "")) isXGridVisible = !isXGridVisible;
            if (ImGui::MenuItem(isYGridVisible?"Hide Grid (Y)":"Show Grid (Y)", "")) isYGridVisible = !isYGridVisible;
            if (ImGui::MenuItem(isZGridVisible?"Hide Grid (Z)":"Show Grid (Z)", "")) isZGridVisible = !isZGridVisible;
            ImGui::Separator();
            if (ImGui::MenuItem("Align camera above X axis ", "")) {
                easyCam.setPosition(ofVec3f(d*sqrt(1.0f-cameraHeightRatio*cameraHeightRatio), cameraHeightRatio*d, 0)+easyCamTarget);
                easyCam.setTarget(easyCamTarget);
            }
            if (ImGui::MenuItem("Align camera above Z axis ", "")) {
                easyCam.setPosition(ofVec3f(0, cameraHeightRatio*d, d*sqrt(1.0f-cameraHeightRatio*cameraHeightRatio))+easyCamTarget);
                easyCam.setTarget(easyCamTarget);
                cout <<"here";
            }
            ImGui::Separator();
            if (ImGui::MenuItem("Align camera along X axis ", "")) {
                easyCam.setPosition(ofVec3f(d, 0, 0)+easyCamTarget);
                easyCam.setTarget(easyCamTarget);
            }
            if (ImGui::MenuItem("Align camera along Y axis ", "")) {
                easyCam.setPosition(ofVec3f(0.001, d, 0)+easyCamTarget);
                easyCam.setTarget(easyCamTarget);
            }
            if (ImGui::MenuItem("Align camera along Z axis ", "")) {
                easyCam.setPosition(ofVec3f(0, 0, d)+easyCamTarget);
                easyCam.setTarget(easyCamTarget);
            }
            
            ImGui::EndMenu();
        }
        
        if (ImGui::BeginMenu("Application")) {
            if (ImGui::MenuItem("Add application specific menu items here", "")) {
            }
            ImGui::EndMenu();
        }

        ImGui::EndMainMenuBar();
    }
}


void ofApp::drawMainWindow() {
    

    ImGui::SetNextWindowPos(ImVec2(0,20), ImGuiSetCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(300,300), ImGuiSetCond_FirstUseEver);
    if (ImGui::Begin("Main")) {

        if (ImGui::CollapsingHeader("3D")) {
            if(ImGui::Button("Reset##CameraTarget")) {
                easyCamTarget = ofVec3f(0,5,0);
                easyCam.setTarget(easyCamTarget);
            }

            ImGui::SameLine();
            if (ImGui::InputFloat3("Camera Target", &easyCamTarget.x)) {
                easyCam.setTarget(easyCamTarget);
            }
            if (ImGui::SliderFloat("Camera Height Ratio", &cameraHeightRatio, 0.0f, 1.0f))
                cameraHeightRatioChanged(cameraHeightRatio);
            ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
        }

        if(ImGui::Button("Reset")) reset();
        ImGui::SameLine();
        if(ImGui::Button(isRunning?"Stop":" Go ")) isRunning = !isRunning;
        ImGui::SameLine();
        ImGui::Text("   Time = %8.1f", t);
        if(ImGui::Button("Quit")) quit();
		if (ImGui::SliderInt("Num Balls", &numBalls, MIN_BALLS, MAX_BALLS)) {	//slider to set the number of balls spawned
			reset();
		}
		if (ImGui::SliderInt("Num Perturb Balls", &numBallsPerturbed, MIN_BALLS, numBalls - 1)) {	//slider to set num balls at angle
			reset();
		}
		if (ImGui::SliderFloat("Perturb Angle", &perturbAngle, MIN_ANGLE, MAX_ANGLE)) {	//slider to set the angle the balls pulled back at
			reset();
		}
		if (ImGui::SliderFloat("Gap", &gap, 0.0f, 1.0f)) {	//slider to set the gap between the balls 
			reset();
		}
        if (ImGui::CollapsingHeader("Numerical Output")) {
            // TODO - numeric output goes here
        }
        
        if (ImGui::CollapsingHeader("Graphical Output")) {
            // TODO - graphical output goes here
        }
    }
    
    // store window size so that camera can ignore mouse clicks
    mainWindowRectangle.setPosition(ImGui::GetWindowPos().x,ImGui::GetWindowPos().y);
    mainWindowRectangle.setSize(ImGui::GetWindowWidth(), ImGui::GetWindowHeight());
    ImGui::End();

}


void ofApp::drawLoggingWindow() {
    ImGui::SetNextWindowSize(ImVec2(290,300), ImGuiSetCond_Always);
    ImGui::SetNextWindowPos(ImVec2(ofGetWindowWidth()-300,20), ImGuiSetCond_Always);
    
    if (ImGui::Begin("Logging")) {
    }
    // store window size so that camera can ignore mouse clicks
    loggingWindowRectangle.setPosition(ImGui::GetWindowPos().x,ImGui::GetWindowPos().y);
    loggingWindowRectangle.setSize(ImGui::GetWindowWidth(), ImGui::GetWindowHeight());
    ImGui::End();
}

//--------------------------------------------------------------
// GUI events and listeners
//--------------------------------------------------------------

void ofApp::keyPressed(int key) {
    
    float d = easyCam.getDistance();
    
    switch (key) {
        
//        case 'h':                               // toggle GUI/HUD
//           isGuiVisible = !isGuiVisible;
//            break;
//        case 'b':                               // toggle debug
//            isDebugVisible = !isDebugVisible;
//            break;
//        case 'a':                               // toggle axis unit vectors
//            isAxisVisible = !isAxisVisible;
//            break;
//        case 'g':                               // toggle ground
//            isGroundVisible = !isGroundVisible;
//            break;
//        case 'u':                               // set the up vecetor to be up (ground to be level)
//            easyCam.setTarget(ofVec3f::zero());
//            break;
//
//        case 'S' :                              // save gui parameters to file
//            gui.saveToFile("settings.xml");
//
//            break;
//        case 'L' :                              // load gui parameters
//            gui.loadFromFile("settings.xml");
//            break;
//
        case 'z':
            easyCam.setPosition(ofVec3f(0, cameraHeightRatio*d, d*sqrt(1.0f-cameraHeightRatio*cameraHeightRatio))+easyCamTarget);
            easyCam.setTarget(easyCamTarget);
            break;
        case 'Z':
            easyCam.setPosition(0, 0, d);
            easyCam.setTarget(ofVec3f::zero());
            break;
        case 'x':
            easyCam.setPosition(ofVec3f(d*sqrt(1.0f-cameraHeightRatio*cameraHeightRatio), cameraHeightRatio*d, 0)+easyCamTarget);
            easyCam.setTarget(easyCamTarget);
            break;
        case 'X':
            easyCam.setPosition(ofVec3f(d, 0, 0)+easyCamTarget);
            easyCam.setTarget(easyCamTarget);
            break;
        case 'Y':
            easyCam.setPosition(ofVec3f(0.001, d, 0)+easyCamTarget);
            easyCam.setTarget(easyCamTarget);
            break;
            
        case 'f':                               // toggle fullscreen
            // BUG: window size is not preserved
            isFullScreen = !isFullScreen;
            if (isFullScreen) {
                ofSetFullscreen(false);
            } else {
                ofSetFullscreen(true);
            }
            break;

        // simulation specific stuff goes here

    }
}


void ofApp::cameraHeightRatioChanged(float & cameraHeightRatio) {
    
    float d = easyCam.getDistance();
    easyCam.setPosition(ofVec3f(0, cameraHeightRatio*d, d*sqrt(1.0f-cameraHeightRatio*cameraHeightRatio))+easyCamTarget);
    easyCam.setTarget(easyCamTarget);
}


void ofApp::quit() {
    ofExit();
}

//--------------------------------------------------------------
// Unused
//--------------------------------------------------------------
void ofApp::keyReleased(int key) {}
void ofApp::mouseMoved(int x, int y ) {}
void ofApp::mouseDragged(int x, int y, int button) {}
void ofApp::mousePressed(int x, int y, int button) {
    // easy camera should ignore GUI mouse clicks
    if (mainWindowRectangle.inside(x,y)||loggingWindowRectangle.inside(x,y))
        easyCam.disableMouseInput();
    else
        easyCam.enableMouseInput();
}
void ofApp::mouseReleased(int x, int y, int button) {}
void ofApp::mouseEntered(int x, int y) {}
void ofApp::mouseExited(int x, int y) {}
void ofApp::windowResized(int w, int h) {}
void ofApp::gotMessage(ofMessage msg) {}
void ofApp::dragEvent(ofDragInfo dragInfo) {}
