#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){

	ofSetWindowShape(1200, 800);

	//Kinects 1
	ofLogNotice("Setup", "CamControl Init");
	ofLogNotice("Setup", "Num Kinect1 sensors = %i",ofxKinect::numAvailableDevices());

	for (int i = 0; i < NUM_KINECTS1; i++)
	{
		ofxKinect k;
		bool initResult = k.init(true,false,true);
		k.open(i);
		ofLogNotice("Setup", "Opening Kinect %i, connected / deviceConnected ? %i / %i", i, k.isConnected(),ofxKinect::isDeviceConnected(i));
		k1List.push_back(&k);
	}

	//Kinect 2
	k2.open();
	k2.initDepthSource();
	k2.initColorSource();
	k2.initInfraredSource();
	k2.initBodySource();
	k2.initBodyIndexSource();


	//RealSense
	rs = ofxRSSDK::RSDevice::createUniquePtr();
	rsIsInit = rs->init();
	if (rsIsInit)
	{
		rs->initDepth(ofxRSSDK::DepthRes::F200_VGA, 60, false);
		rs->enablePointCloud(ofxRSSDK::CloudRes::HALF_RES, 100, 1000);

		rsDepthTex.allocate(rs->getDepthWidth(), rs->getDepthHeight(), GL_RGBA);
		rsIsStarted = rs->start();

		if (!rsIsStarted)
		{
			ofLogNotice("Setup","RealSense is init but could not start.");
		}
	} else
	{
		ofLogNotice("Setup","RealSense init error");
	}

	setupCamera();
}

//--------------------------------------------------------------
void ofApp::update(){
	if (k2.isOpen())
	{
		k2.update();
	}

	if (rsIsInit && rsIsStarted)
	{
		rs->update();
		rsDepthTex.loadData(rs->getDepth8uFrame());

		rsCloudMesh.clear();
		rsCloudMesh.setMode(OF_PRIMITIVE_POINTS);
		//rsCloudMesh.enableColors();

		//TODO: Figure out a better way to work with BGRA pixels
		vector<ofVec3f> rsPointCloud = rs->getPointCloud();
		for (vector<ofVec3f>::iterator p = rsPointCloud.begin(); p != rsPointCloud.end(); ++p)
		{
			rsCloudMesh.addVertex(*p);
			//ofColor c = rs->getColorFromDepthSpace(*p);
			//mCloudMesh.addColor(ofColor(c.b, c.g, c.r));
		}
	}
}

//--------------------------------------------------------------
void ofApp::draw(){

	ofClear(ofColor::black);
	ofSetColor(ofColor::white);


	int imgSize = 400;


	for (int i = 0; i < NUM_KINECTS1; i++)
	{
		ofxKinect * k = k1List[i];

		bool kIsConnected = ofxKinect::isDeviceConnected(i) && k->isConnected();
		if (kIsConnected)
		{
			float tx = i % 2;
			float ty = floor(i / 2);
			k->drawDepth(ofRectangle(tx, ty, imgSize / 2, imgSize / 2));
		} else
		{
			//ofLogNotice("Kinect " + ofToString(i) + " is not connected, not drawing.");
		}
	}

	
	if (k2.isOpen() && k2.isFrameNew())
	{
		k2.getDepthSource()->draw(imgSize * 2, 0, imgSize, imgSize);
	} else
	{
		//ofLogNotice("Kinect2 is not open or no new frame");
	}


	if (rsIsInit && rsIsStarted)
	{
		rsDepthTex.draw(imgSize * 2, imgSize, imgSize, imgSize);
		
	}


	//3D Rendering
	mCamera.begin();

	if (rsIsInit && rsIsStarted)
	{
		rsCloudMesh.draw();
	}

	mCamera.end();
}

void ofApp::exit()
{

	for (int i = 0; i < NUM_KINECTS1; i++)
	{
		bool kIsConnected = ofxKinect::isDeviceConnected(i) && k1List[i]->isConnected();
		if (kIsConnected)
		{
			k1List[i]->close();
			delete k1List[i];
		}
	}
	if (k2.isOpen())
	{
		k2.close();
	}

	if (rsIsInit && rsIsStarted)
	{
		rs->stop();
	}
}


//----- CAMERA

void ofApp::setupCamera()
{
	mCamera.setFov(45.0f);
	mCamera.setAspectRatio(ofGetWindowWidth() / (float)ofGetWindowHeight());
	mCamera.setNearClip(100);
	mCamera.setFarClip(5000);

	mCamera.setGlobalPosition(ofVec3f(0, 0, 0));
	mCamera.lookAt(ofVec3f(0, 0, 100), ofVec3f(0, 1, 0));
	mCamera.setAutoDistance(true);
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

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
