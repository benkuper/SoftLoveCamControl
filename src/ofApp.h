#pragma once

#include "ofMain.h"
#include "ofxKinectForWindows2.h"
#include "ofxKinect.h"
#include "ofxRSSDKv2.h"

#include "ofxSharedMemory.h"

#include "ofxOsc.h"

#define NUM_KINECTS1 3

#define K1_PCL_WIDTH 320
#define K1_PCL_HEIGHT 240
#define K2_PCL_WIDTH 512
#define K2_PCL_HEIGHT 424
#define RS_PCL_WIDTH 640
#define RS_PCL_HEIGHT 480

#define NUM_K1_PIXELS K1_PCL_WIDTH*K1_PCL_HEIGHT
#define NUM_K2_PIXELS K2_PCL_WIDTH*K2_PCL_HEIGHT
#define NUM_RS_PIXELS RS_PCL_WIDTH*RS_PCL_HEIGHT

#include "PCLData.h"

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
		int k1Steps;
		vector<ofxKinect *> k1List;

		//KINECT 2
		int k2Steps;
		ofxKFW2::Device k2;
		ICoordinateMapper * k2Mapper;
		ofTexture k2Depthtex;
		ofVec3f k2TmpCloud[NUM_K2_PIXELS];

		//REALSENSE
		int rsSteps;
		ofxRSSDK::RSDevicePtr rs;
		bool rsIsInit;
		bool rsIsStarted; 
		ofTexture rsDepthTex;

		//PCL DATA & MEMORY SHARE
		PCLData * pclData; 
		ofxSharedMemory<PCLData*> memoryMappedFile;
		string memoryKey = "SoftLovePCL";
		int memorySize = sizeof(PCLData);
		bool memoryIsConnected = false;

		//OSC
		int port = 6001;
		ofxOscReceiver oscReceiver;
		void processOSC();

};
