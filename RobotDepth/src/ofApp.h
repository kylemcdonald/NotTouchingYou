#pragma once

//--------------------------------------------------------------
//
//
// Basic Move Example
//
//
//--------------------------------------------------------------

#include "ofMain.h"
#include "ofxGui.h"
#include "ofxGizmo.h"
#include "RobotController.h"
#include "RobotParameters.h"
#include "URIKFast.h"
#include "ofxGui.h"
#include "ofxVirtualKinect.h"
#include "ofxCv.h"

#define N_CAMERAS 2

class ofApp : public ofBaseApp{
    
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
    
    RobotParameters parameters;
    
    URIKFast kinematics;
    
    ofxGizmo gizmo;
    ofNode tcpNode;
    
    ofParameterGroup appParams;
    ofParameter<float> orthoScaleRobot;
    ofParameter<float> xOffset, yOffset, zOffset, zRotation;
    ofParameter<bool> codeGesture;
    bool lastCodeGesture;
    Joint previousTCP;
    
    void setupViewports();
    void setupGUI();
    void positionGUI();
    void drawGUI();
    
    ofxPanel panel;
    ofxPanel panelJoints;
    ofxPanel panelTargetJoints;
    ofxPanel panelJointsIK;
    ofxPanel panelJointsSpeed;
    
    
    RobotController robot;
    void moveTCP();
    
    
    // 3D Navigation
    void updateActiveCamera();
    vector<ofEasyCam*> cams;
    ofRectangle viewportReal;
    ofRectangle viewportSim;
    ofRectangle viewportDepth;
    vector<ofMatrix4x4> savedCamMats;
    vector<string> viewportLabels;
    int activeCam;
    
    /**
     Use hotkeys to cyle through preset viewports.
     @param key
     '1' = Top View      <br/>
     '2' = Left View     <br/>
     '3' = Front View    <br/>
     '4' = Perspective   <br/>
     */
    void handleViewportPresets(int key);
    
    // from VirtualKinect example
    ofxVirtualKinect kinect;
    ofParameter<ofVec3f> position, cameraRotation;
    ofParameter<float> near, far, maxLength, orthoScaleKinect;
    ofParameter<int> stepSize;
    ofParameter<bool> horizontalFlip;
    void updateKinect();
    
    ofParameter<float> tcpHeight;
    ofParameter<bool> mouseFollow, kinectFollow;
    ofParameter<ofVec3f> mousePositionRobot, closestPointRobot;
    
    ofxCv::ContourFinder contourFinder;
    ofVec2f closestPointKinect;
    ofParameter<float> minContourArea;
    ofParameter<float> maintainDistance;
    
    ofVec2f convertRobotToKinect(ofVec2f v) {
        v.y *= -1; // ofRotateX(180);
        v.rotate(zRotation); // ofRotateZ(zRotation);
        v += ofVec2f(xOffset, yOffset); // ofTranslate(xOffset, yOffset, zOffset);
        v *= orthoScaleRobot; // ofScale(orthoScaleRobot, orthoScaleRobot, -1);
        v += ofVec2f(320, 240); // ofTranslate(w/2, h/2);
        return v;
    }
    
    // need to check this
    ofVec2f convertKinectToRobot(ofVec2f v) {
        v -= ofVec2f(320, 240); // ofTranslate(w/2, h/2);
        v /= orthoScaleRobot; // ofScale(orthoScaleRobot, orthoScaleRobot, -1);
        v -= ofVec2f(xOffset, yOffset); // ofTranslate(xOffset, yOffset, zOffset);
        v.rotate(-zRotation); // ofRotateZ(zRotation);
        v.y *= -1; // ofRotateX(180);
        return v;
    }
};
