//Copyright (c) 2016, Daniel Moore, Madaline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.

//--------------------------------------------------------------
//
//
// Basic Move Example
//
//
//--------------------------------------------------------------

//
// This example shows you how to:
//  1.  Connect ofxRobotArm to your robot via ethernet
//  2.  Move & reiorient the simulated robot (dragging ofxGizmo)
//  3.  Move & reiorient the real-time robot (use 'm' to enable real-time movement)
//
// Remeber to swap in your robot's ip address in robot.setup() [line 40]
// If you don't know your robot's ip address, you may not be set up yet ...
//  -   Refer to http://www.universal-robots.com/how-tos-and-faqs/how-to/ur-how-tos/ethernet-ip-guide-18712/
//      for a walk-thru to setup you ethernet connection
// See the ReadMe for more tutorial details


#include "ofApp.h"

using namespace ofxCv;

//--------------------------------------------------------------
void ofApp::setup(){
    ofSetFrameRate(60);
    ofSetVerticalSync(true);
    ofBackground(0);
//    ofSetLogLevel(OF_LOG_SILENT);
    
    kinect.setup();
    
    lastCodeGesture = false;
    appParams.setName("App Parameters");
    
    setupViewports();
    
    parameters.setup();
    robot.setup("192.168.1.9", parameters); // <-- change to your robot's ip address
    robot.disableControlJointsExternally();
    
    contourFinder.setSortBySize(true);
    
    setupGUI();
    positionGUI();
}

//--------------------------------------------------------------
void ofApp::update(){
    updateKinect();
    moveTCP();
    robot.update();
    updateActiveCamera();
}

void ofApp::updateKinect() {
    kinect.setPosition(position.get());
    kinect.setCameraRotation(cameraRotation.get());
    kinect.setClipping(near, far);
    kinect.setStepSize(stepSize);
    kinect.setMaxLength(maxLength);
    kinect.setOrthoScale(orthoScaleKinect);
    kinect.setHorizontalFlip(horizontalFlip);
    
    kinect.update();
    
    if(kinect.isFrameNew()) {
        contourFinder.setThreshold(contourThreshold);
        contourFinder.setMinAreaRadius(minContourArea);
        contourFinder.findContours(kinect.getPixels());
    }
}

//--------------------------------------------------------------
void ofApp::draw(){
    
    float depthImageScale = 2;
    
    // mouse following conversion
    ofVec2f mouseRaw(mouseX, mouseY);
    mouseRaw /= depthImageScale;
    ofVec2f m2d = convertKinectToRobot(mouseRaw);
    
    // kinect following conversion
    if(contourFinder.size() > 0) {
        ofPolyline& blob = contourFinder.getPolyline(0);
        // look for the point that's closest to the robot's position (xOffset, yOffset)
        float minDistance = 0;
        ofVec2f curClosest;
        ofVec2f center(xOffset, yOffset);
        int n = blob.size();
        for(int i = 0; i < n; i++) {
            float distance = center.distance(blob[i]);
            if(i == 0 || distance < minDistance) {
                minDistance = distance;
                curClosest = blob[i];
            }
        }
        
        // always interpolate position
        closest.interpolate(curClosest, kinectLerpAmount);
        
        float sampleOffset = 6;
        samplePoint = closest + (closest - center).normalize() * sampleOffset;
        int samplex = round(samplePoint.x);
        int sampley = round(samplePoint.y);
        float depthSum = 0;
        int depthCount = 0;
        int sampleRadius = 4;
        unsigned char* kinectPixels = kinect.getPixels().getData();
        int kinectHeight = kinect.getHeight();
        int kinectWidth = kinect.getWidth();
        for(int yoff = -sampleRadius; yoff <= +sampleRadius; yoff++) {
            for(int xoff = -sampleRadius; xoff <= +sampleRadius; xoff++) {
                int x = ofClamp(samplex + xoff, 0, kinectWidth-1);
                int y = ofClamp(sampley + yoff, 0, kinectHeight-1);
                int i = y * kinectWidth + x;
                if(kinectPixels[i] > contourThreshold) {
                    depthSum += kinect.grayToDistance(kinectPixels[i]);
                    depthCount++;
                }
            }
        }
        
        if(depthCount > 0) {
            float curVisitorHeight = sensorHeight - (depthSum / depthCount);
            // always interpolate height
            visitorHeight = ofLerp(visitorHeight, curVisitorHeight, kinectLerpAmount);
        } else {
//            ofLog() << "Not enough depth samples: " << depthCount;
        }
    }
    ofVec2f k2d = convertKinectToRobot(closest);
    
    float z;
    if(kinectFollow) {
        p2d = k2d;
        z = visitorHeight - armHeight;
    }
    if(mouseFollow) {
        p2d = m2d;
        z = tcpHeight;
    }
    ofVec2f p2dpre = p2d;
    ofVec2f pdir = p2d.getNormalized();
    p2d -= pdir * maintainDistance;
    if(kinectFollow || mouseFollow) {
        float baseRotationOffset = +90; // accounts for base rotation offset
        float minRadius = 320, maxRadius = 600;
        float minHeight = -500, maxHeight = +600;
        float thetaConstraint = 0.75;
        float leadRange = 45; // max degrees to lead with gaze
        float t = ofGetElapsedTimef() * positionNoiseSpeed;
        
        // clamp rotation about center
        p2d.rotate(+baseRotationOffset); // apply rotation
        float theta = atan2(p2d.y, p2d.x);
        theta = ofClamp(theta, -PI * thetaConstraint, +PI * thetaConstraint);
        p2d.rotate(-baseRotationOffset); // unapply rotation
        
        // clamp distance from center
        float radius = sqrt(p2d.x * p2d.x + p2d.y * p2d.y);
        radius = ofClamp(radius, minRadius, maxRadius);
        
        // set rotation vector based on unclamped position
        ofQuaternion rotate;
        ofVec2f target = ofVec3f(parameters.tcpPosition) * 1000;
        float baseTheta = baseRotationOffset - ofRadToDeg(theta);
        float leadingTheta = -ofRadToDeg(atan2(p2dpre.y - target.y, p2dpre.x - target.x));
        float clampedTheta = baseTheta + ofClamp(leadingTheta - baseTheta, -leadRange, +leadRange);
        float xrot = ofSignedNoise(t, 0) * rotationNoise;
        float yrot = ofSignedNoise(0, t) * rotationNoise;
        ofVec3f up(xrot,yrot,-1); // up vector points -z
        rotate.makeRotate(clampedTheta, up);
        parameters.targetTCP.rotation = rotate;
        
        // clamp position and unapply rotation
        p2d.x = cos(theta) * radius;
        p2d.y = sin(theta) * radius;
        p2d.rotate(-baseRotationOffset); // unapply rotation
        
        float clampedHeight = ofClamp(z, minHeight, maxHeight);
        
        // set position to constrained location
        ofVec3f noise(ofSignedNoise(t, 0, 0),
                      ofSignedNoise(0, t, 0),
                      ofSignedNoise(0, 0, t));
        noise *= positionNoise;
        noise *= ofMap(sin((ofGetElapsedTimef() * TWO_PI) / breathRate), -1, +1, 0, 1);
        mousePositionRobot.set(ofVec3f(p2d.x, p2d.y, clampedHeight) + noise);
        parameters.targetTCP.position = mousePositionRobot.get() / 1000.;
    }
    
    gizmo.setViewDimensions(viewportSim.width, viewportSim.height);
    
    // show realtime robot
    cams[0]->begin(viewportSim);
    ofPushStyle();
    tcpNode.draw();
    
    ofDisableDepthTest();
    ofSetColor(magentaPrint, 128);
    robot.draw();
    
    gizmo.draw(*cams[0]);
    ofSetColor(255, 128);
    robot.drawPreview();
    
    ofNoFill();
    ofSetColor(yellowPrint);
    ofDrawSphere(mousePositionRobot, 10);
    ofSetColor(cyanPrint);
    ofDrawSphere(closestPointRobot, 10);
    ofPopStyle();
    cams[0]->end();
    
    ofFbo& robotMaskImage = kinect.fbo;
    robotMaskImage.begin();
    
    float w = robotMaskImage.getWidth();
    float h = robotMaskImage.getHeight();
    
    float maxClip = 2000;
    ofSetupScreenOrtho(w, h, -maxClip, +maxClip);
    
    ofTranslate(w/2, h/2);
    ofScale(orthoScaleRobot, orthoScaleRobot, -1);
    ofTranslate(xOffset, yOffset, zOffset);
    ofRotateZ(zRotation);
    ofRotateX(180);
    
    ofSetColor(0); // black out robot
    robot.draw(false);
    
    robotMaskImage.end();
    
    ofPushMatrix();
    ofTranslate(viewportSim.width, 0);
    ofPushMatrix();
    ofScale(depthImageScale, depthImageScale);
    
    ofPushStyle();
    
    ofSetColor(255);
    kinect.draw(0, 0);
    ofNoFill();
    contourFinder.draw();
    if(contourFinder.size() > 0) {
        ofSetColor(magentaPrint);
        contourFinder.getPolyline(0).draw();
        ofSetColor(yellowPrint);
        ofDrawCircle(samplePoint, 3);
        ofSetColor(cyanPrint);
        ofDrawCircle(closest, 8);
        ofSetColor(255);
    }
    
    ofNoFill();
    
    ofSetColor(cyanPrint);
    ofDrawCircle(mouseRaw, 8);
    
    ofSetColor(magentaPrint);
    ofVec3f target = ofVec3f(parameters.tcpPosition) * 1000;
    ofDrawCircle(convertRobotToKinect(target), 8);
    
    ofSetColor(yellowPrint);
    ofVec3f actual = ofVec3f(parameters.targetTCPPosition) * 1000;
    ofDrawCircle(convertRobotToKinect(actual), 8);
    
    ofDrawRectangle(0, 0, w, h);
    
    ofPopStyle();
    ofPopMatrix();
    ofPopMatrix();
    
    drawGUI();
}

void ofApp::moveTCP(){
    
    if(codeGesture) {
        if(!lastCodeGesture) {
            previousTCP = parameters.targetTCP;
        }
        float r = 100;
        float speed = 1.0 * TWO_PI;
        parameters.targetTCP.position.x = previousTCP.position.x;
        parameters.targetTCP.position.y = previousTCP.position.y + (r * fmodf(ofGetElapsedTimef(), 1)) / 1000;
//        parameters.targetTCP.position.z = previousTCP.position.z;
    }
    lastCodeGesture = codeGesture;
    
    // assign the target pose to the current robot pose
    if(parameters.bCopy){
        parameters.bCopy = false;
        
        parameters.bCopy = false;
        parameters.targetTCP.rotation = ofQuaternion(90, ofVec3f(0, 0, 1));
        parameters.targetTCP.rotation*=ofQuaternion(90, ofVec3f(1, 0, 0));
        
        // get the robot's position
        parameters.targetTCP.position = parameters.actualTCP.position;
        parameters.targetTCP.rotation*=parameters.actualTCP.rotation;
        
        tcpNode.setPosition(parameters.targetTCP.position*1000);
        tcpNode.setOrientation(parameters.targetTCP.rotation);
        gizmo.setNode(tcpNode);
        // update GUI params
        parameters.targetTCPPosition = parameters.targetTCP.position;
        parameters.targetTCPOrientation = ofVec4f(parameters.targetTCP.rotation.x(), parameters.targetTCP.rotation.y(), parameters.targetTCP.rotation.z(), parameters.targetTCP.rotation.w());
        
    }
    else{
        tcpNode.setTransformMatrix(gizmo.getMatrix());
    }
    
    // follow the gizmo's position and orientation
    if(parameters.bFollow){
        parameters.targetTCP.position.interpolate(tcpNode.getPosition()/1000.0, parameters.followLerp);
        parameters.targetTCP.rotation = tcpNode.getOrientationQuat();
        parameters.targetTCPOrientation = ofVec4f(parameters.targetTCP.rotation.x(), parameters.targetTCP.rotation.y(), parameters.targetTCP.rotation.z(), parameters.targetTCP.rotation.w());
        
    }
}

void ofApp::setupViewports(){
    viewportSim = ofRectangle(0, 0, ofGetWidth()/2, ofGetHeight());
    viewportReal = ofRectangle(0, 0, 1, 1);
    viewportDepth = ofRectangle(0, 0, 640, 480);
    
    activeCam = 0;
    
    
    for(int i = 0; i < N_CAMERAS; i++){
        cams.push_back(new ofEasyCam());
        savedCamMats.push_back(ofMatrix4x4());
        viewportLabels.push_back("");
    }
    
    cams[0]->begin(viewportSim);
    cams[0]->end();
    cams[0]->enableMouseInput();
    
    
    cams[1]->begin(viewportReal);
    cams[1]->end();
    cams[1]->enableMouseInput();
    
}

//--------------------------------------------------------------
void ofApp::setupGUI(){
    
    panel.setup(parameters.robotArmParams);
    
    panel.add(parameters.pathRecorderParams);
    
    panelJoints.setup(parameters.joints);
    panelTargetJoints.setup(parameters.targetJoints);
    panelJointsSpeed.setup(parameters.jointSpeeds);
    panelJointsIK.setup(parameters.jointsIK);
    
    panel.add(robot.movement.movementParams);
    parameters.bMove = false;
    // get the current pose on start up
    parameters.bCopy = true;
    panel.loadFromFile("settings/settings.xml");
    
    // setup Gizmo
    gizmo.setDisplayScale(1.0);
    tcpNode.setPosition(ofVec3f(0.5, 0.5, 0.5)*1000);
    tcpNode.setOrientation(parameters.targetTCP.rotation);
    gizmo.setNode(tcpNode);
    
    // setup robot depth
    float clipRange = 1000;
    appParams.add(orthoScaleRobot.set("ortho scale robot", 0.25, 0, 1));
    
    appParams.add(xOffset.set("x offset", 887, -1000, +1000));
    appParams.add(yOffset.set("y offset", 19, -50, +50));
    appParams.add(zOffset.set("z offset", 0, -1000, +1000));
    appParams.add(zRotation.set("z rotation", 90, -180, +180));
    appParams.add(codeGesture.set("code gesture", lastCodeGesture));
    
    // setup virtual kinect
    float maxPosition = 2000;
    appParams.add(position.set("position", ofVec3f(-410, 62, 0), ofVec3f(-maxPosition, -100, -100), ofVec3f(+maxPosition, +100, +100)));
    appParams.add(cameraRotation.set("camera rotation", ofVec3f(0, 0, -2), ofVec3f(1,1,1) * -10, ofVec3f(1,1,1) * +10));
    appParams.add(near.set("near", 2000, 2000, 4000));
    appParams.add(far.set("far", 3800, 2000, 4000));
    appParams.add(maxLength.set("maxLength", 200, 0, 400));
    appParams.add(stepSize.set("stepSize", 2, 1, 16));
    appParams.add(orthoScaleKinect.set("ortho scale kinect", 4, 1, 6));
    appParams.add(horizontalFlip.set("horizontal flip", false));
    
    appParams.add(tcpHeight.set("tcp height", 330, -500, 500));
    appParams.add(mousePositionRobot.set("mouse position",
                                        ofVec3f(),
                                        ofVec3f(1,1,1) * -1000,
                                        ofVec3f(1,1,1) * +1000));
    appParams.add(mouseFollow.set("mouse follow", false));
    appParams.add(closestPointRobot.set("closest point",
                                        ofVec3f(),
                                        ofVec3f(1,1,1) * -1000,
                                        ofVec3f(1,1,1) * +1000));
    appParams.add(kinectFollow.set("kinect follow", false));
    appParams.add(contourThreshold.set("contour threshold", 40, 0, 255));
    appParams.add(minContourArea.set("min contour area", 20, 0, 40));
    appParams.add(maintainDistance.set("maintain dist mm", 100, 0, 1000));
    appParams.add(sensorHeight.set("sensor height mm", 4100, 4000, 4200));
    appParams.add(visitorHeight.set("visitor height mm", 1600, 0, 2000));
    appParams.add(armHeight.set("arm height mm", 990, 500, 1500));
    appParams.add(kinectLerpAmount.set("lerp amount", .1, 0, 1));
    appParams.add(positionNoise.set("position noise mm", 15, 0, 50));
    appParams.add(rotationNoise.set("rotation noise", .1, 0, .5));
    appParams.add(positionNoiseSpeed.set("position noise speed", .1, 0, 1));
    appParams.add(breathRate.set("breath rate s", 2, .5, 5));
    panel.add(appParams);
}

void ofApp::positionGUI(){
    panel.setPosition(viewportSim.x+viewportSim.width, 10);
    panelJointsSpeed.setPosition(viewportReal.x, 10);
    panelJointsIK.setPosition(panelJointsSpeed.getPosition().x+panelJoints.getWidth(), 10);
    panelTargetJoints.setPosition(panelJointsIK.getPosition().x+panelJoints.getWidth(), 10);
    panelJoints.setPosition(panelTargetJoints.getPosition().x+panelJoints.getWidth(), 10);
}

//--------------------------------------------------------------
void ofApp::drawGUI(){
    panel.draw();
    panelJoints.draw();
    panelJointsIK.draw();
    panelJointsSpeed.draw();
    panelTargetJoints.draw();
    
    ofPushStyle();
    ofSetColor(255,160);
    ofDrawBitmapString("OF FPS "+ofToString(ofGetFrameRate()), 30, ofGetWindowHeight()-50);
    ofDrawBitmapString("Robot FPS "+ofToString(robot.robot.getThreadFPS()), 30, ofGetWindowHeight()-65);
    ofPopStyle();
}


//--------------------------------------------------------------
void ofApp::updateActiveCamera(){
    
    if(viewportSim.inside(ofGetMouseX(), ofGetMouseY()))
    {
        activeCam = 0;
        if(!cams[0]->getMouseInputEnabled()){
            cams[0]->enableMouseInput();
        }
        if(gizmo.isInteracting() && cams[0]->getMouseInputEnabled()){
            cams[0]->disableMouseInput();
        }
    }
}

//--------------------------------------------------------------
//--------------------------------------------------------------
void ofApp::handleViewportPresets(int key){
    
    float dist = 2000;
    float zOffset = 450;
    
    if(activeCam != -1){
        // TOP VIEW
        if (key == '1'){
            cams[activeCam]->reset();
            cams[activeCam]->setPosition(0, 0, dist);
            cams[activeCam]->lookAt(ofVec3f(0, 0, 0), ofVec3f(0, 0, 1));
            viewportLabels[activeCam] = "TOP VIEW";
        }
        // LEFT VIEW
        else if (key == '2'){
            cams[activeCam]->reset();
            cams[activeCam]->setPosition(dist, 0, 0);
            cams[activeCam]->lookAt(ofVec3f(0, 0, 0), ofVec3f(0, 0, 1));
            viewportLabels[activeCam] = "LEFT VIEW";
        }
        // FRONT VIEW
        else if (key == '3'){
            cams[activeCam]->reset();
            cams[activeCam]->setPosition(0, dist, 0);
            cams[activeCam]->lookAt(ofVec3f(0, 0, 0), ofVec3f(0, 0, 1));
            viewportLabels[activeCam] = "FRONT VIEW";
        }
        // PERSPECTIVE VIEW
        else if (key == '4'){
            cams[activeCam]->reset();
            cams[activeCam]->setPosition(dist, dist, dist/4);
            cams[activeCam]->lookAt(ofVec3f(0, 0, 0), ofVec3f(0, 0, 1));
            viewportLabels[activeCam] = "PERSPECTIVE VIEW";
        }
    }
}


//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    
    if(key == 'm') {
        parameters.bMove = !parameters.bMove;
    }
    
    if( key == 'r' ) {
        gizmo.setType( ofxGizmo::OFX_GIZMO_ROTATE );
    }
    if( key == 'g' ) {
        gizmo.setType( ofxGizmo::OFX_GIZMO_MOVE );
    }
    if( key == 'e' ) {
        gizmo.toggleVisible();
    }
    
    handleViewportPresets(key);
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){
    
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){
    
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){
    viewportLabels[activeCam] = "";
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){
    
}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){
    
}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){
    
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){
    
}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 
    
}
