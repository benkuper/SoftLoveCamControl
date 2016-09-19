#pragma once

#include "ofMain.h"
#include "ofxKinectForWindows2.h"
#include "ofxKinect.h"
#include "ofxRSSDKv2.h"

#define NUM_KINECTS1 4

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();
		void exit();

		
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

		//KINECTS 1
		vector<ofxKinect *> k1List;

		//KINECT 2
		ofxKFW2::Device k2;


		//REALSENSE
		ofxRSSDK::RSDevicePtr rs;
		bool rsIsInit;
		bool rsIsStarted;
		ofMesh rsCloudMesh;
		ofTexture rsDepthTex;
		

		//CAMERA
		ofEasyCam mCamera;
		void setupCamera();

};
