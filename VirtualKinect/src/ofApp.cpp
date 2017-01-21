#include "ofApp.h"

void ofApp::setup() {
	ofSetVerticalSync(true);
    ofSetFrameRate(60);
    
    kinect.setup();
    
    float maxPosition = 4000;
    panel.setup();
    panel.add(position.set("position", ofVec3f(), ofVec3f(1,1,1) * -maxPosition, ofVec3f(1,1,1) * +maxPosition));
    panel.add(cameraRotation.set("camera rotation", ofVec3f(), ofVec3f(1,1,1) * -180, ofVec3f(1,1,1) * +180));
    panel.add(sceneRotation.set("scene rotation", ofVec3f(), ofVec3f(1,1,1) * -180, ofVec3f(1,1,1) * +180));
    panel.add(near.set("near", 0, 0, maxPosition));
    panel.add(far.set("far", maxPosition, 0, maxPosition));
    panel.add(maxLength.set("maxLength", 200, 0, maxPosition));
    panel.add(stepSize.set("stepSize", 2, 1, 16));
    panel.add(orthoScale.set("ortho scale", 10, 0, 32));
    panel.add(horizontalFlip.set("horizontal flip", false));
}

void ofApp::update() {
	kinect.setPosition(position.get());
	kinect.setCameraRotation(cameraRotation.get());
	kinect.setSceneRotation(sceneRotation.get());
	kinect.setClipping(near, far);
    kinect.setStepSize(stepSize);
    kinect.setMaxLength(maxLength);
    kinect.setOrthoScale(orthoScale);
    kinect.setHorizontalFlip(horizontalFlip);

	kinect.update();
	
	if(kinect.isFrameNew()) {
	}
}

void ofApp::draw() {
	ofBackground(127);
	ofSetColor(255, 255, 255);
	kinect.draw(300, 100);
    panel.draw();
}

void ofApp::exit() {
    kinect.close();
}
