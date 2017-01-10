#pragma once

#include "ofMain.h"
#include "ofxKinectForWindows2.h"
#include "ofxKinect.h"
#include "ofxRSSDKv2.h"

#include "ofxSharedMemory.h"

#include "ofxOsc.h"
#include "ofxSpout.h"

#define NUM_KINECTS1 3

#define K1_PCL_WIDTH 320
#define K1_PCL_HEIGHT 240
#define K2_PCL_WIDTH 512
#define K2_PCL_HEIGHT 424
#define RS_PCL_WIDTH 640
#define RS_PCL_HEIGHT 480

#define K2_COLOR_WIDTH 1920
#define K2_COLOR_HEIGHT 1080

#define NUM_K1_PIXELS K1_PCL_WIDTH*K1_PCL_HEIGHT
#define NUM_K2_PIXELS K2_PCL_WIDTH*K2_PCL_HEIGHT
#define NUM_RS_PIXELS RS_PCL_WIDTH*RS_PCL_HEIGHT

#include "PCLData.h"


/*
DISABLE / ENABLE FAILGUARD ON LIBUSB : check in  ofxKinect/libfreenect/platform/windows/libusbemu/libusbemu.cpp, commented section on line 771
*/


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


		//UI
		bool doDraw;

		//KINECTS 1
		int k1Steps;
		vector<ofxKinect *> k1List;
		bool freezeK1[NUM_KINECTS1];
		ofxSpout::Sender k1Senders[NUM_KINECTS1];

		bool isK1PointGood(int kinectIndex, int pIndex);


		//KINECT 2
		int k2Steps;
		ofxKFW2::Device k2;
		ICoordinateMapper * k2Mapper;
		ofTexture k2DepthTex;
		ofTexture k2ColorTex;
		ofImage  k2ColorImage;
		ColorSpacePoint colorIndices[NUM_K2_PIXELS];
		bool freezeK2;
		ofxSpout::Sender k2Sender;

			//K2 body
		ofTexture k2BodyTex;
		ofPixels k2BodyPixels;
		int numBodiesTracked;

		bool isK2PointGood(int pIndex);
		

		//REALSENSE
		int rsSteps;
		ofxRSSDK::RSDevicePtr rs;
		bool rsIsInit;
		bool rsIsStarted; 
		ofTexture rsDepthTex;
		bool freezeRS;

		//PCL DATA & MEMORY SHARE
		PCLData * pclData; 
		ofxSharedMemory<PCLData*> memoryMappedFile;
		string memoryKey = "SoftLovePCL";
		int memorySize = sizeof(PCLData);
		bool memoryIsConnected = false;
		std::mutex mtx;

		//OSC
		int port = 6001;
		ofxOscReceiver oscReceiver;
		void processOSC();




};
