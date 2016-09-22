#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){

	ofSetWindowShape(600,400);

	// PCL DATA & MEMORY SHARE
	ofLogNotice("Setup", "MemoryShare Init");
	memoryMappedFile.setup(memoryKey, memorySize, true);
	//memoryIsConnected = memoryMappedFile.connect();
	ofLog() << "Memory key: " << memoryKey;
	ofLog() << "Memory size: " << memorySize;
	//ofLog() << "Memory was mapped? " << memoryIsConnected;


	//Kinects 1
	ofLogNotice("Setup", "CamControl Init");
	ofLogNotice("Setup", "Num Kinect1 sensors = %i",ofxKinect::numAvailableDevices());
	

	oscReceiver.setup(port);

	for (int i = 0; i < NUM_KINECTS1; i++)
	{
		ofxKinect * k = new ofxKinect();
		k->setRegistration(true);
		bool initResult = k->init(true,true,true);
		k->open(i);
		ofLogNotice("Setup", "Opening Kinect %i, connected / deviceConnected ? %i / %i", i, k->isConnected(),ofxKinect::isDeviceConnected(i));
		k1List.push_back(k);
		ofSleepMillis(100);
	}

	//Kinect 2
	k2.open();
	k2.initDepthSource();
	k2Mapper = k2.getDepthSource()->getCoordinateMapper();

	//RealSense
	rs = ofxRSSDK::RSDevice::createUniquePtr();
	rsIsInit = rs->init();
	if (rsIsInit)
	{
		rs->initDepth(ofxRSSDK::DepthRes::F200_VGA, 60, true);
		rs->enablePointCloud(ofxRSSDK::CloudRes::HALF_RES, 100, 1000);
		rs->initRgb(ofxRSSDK::RGBRes::VGA, 60);

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

	//define steps
	k1Steps = 2;
	k2Steps = 4;
	rsSteps = 4;
	
}

//--------------------------------------------------------------
void ofApp::update(){

	processOSC();

	//MemoryMap
	ofSetWindowTitle(ofToString("Server connected : "+ ofToString(memoryIsConnected ? "YES" : "NO") + ", FPS: " + ofToString(ofGetFrameRate())));

	// if not connected, try reconnect every 5 seconds or so
	if (!memoryIsConnected && ofGetFrameNum() % 300 == 0) {
		memoryIsConnected = memoryMappedFile.connect();
		if (memoryIsConnected)
		{

			//memoryMappedFile.setData(new CustomData());
			pclData = memoryMappedFile.getData();
		}

	}


	//KINECT 1
	for (int i = 0; i < NUM_KINECTS1; i++)
	{
		ofxKinect * k = k1List[i];
		if (!k->isConnected()) continue;

		k->update();
		
		if (memoryIsConnected)
		{
			int numGoodK1Points = 0;
			int tIndex = 0;

			for (int ty = 0; ty < K1_PCL_HEIGHT; ty+=k1Steps)
			{
				for (int tx = 0; tx < K1_PCL_WIDTH; tx+= k1Steps)
				{
					ofVec3f p = k->getWorldCoordinateAt(tx*2, ty*2)*ofVec3f(1,-1,1)/1000.; //ofVec3f(tx*1./K1_PCL_WIDTH,ty*1./K1_PCL_HEIGHT,.5f);// 

					bool isGood = p.x != 0 && !isnan(p.x);

					if (isGood)
					{
						pclData->k1Clouds[i].points[numGoodK1Points] = p;
						pclData->k1Clouds[i].positions[numGoodK1Points] = ty*K1_PCL_WIDTH+tx;
						numGoodK1Points++;
					}				
				}
			}

			//ofLog() << "num good k1 points : " << numGoodK1Points;
			pclData->k1Clouds[i].numGoodPoints = numGoodK1Points;
		}
		
	}


	// KINECT 2
	
	if (k2.isOpen())
	{
		k2.update();
		if (k2.isFrameNew())
		{
			k2Depthtex = k2.getDepthSource()->getTexture();

			if (memoryIsConnected)
			{
				k2Mapper->MapDepthFrameToCameraSpace(NUM_K2_PIXELS, k2.getDepthSource()->getPixels(), NUM_K2_PIXELS, reinterpret_cast<CameraSpacePoint*>(k2TmpCloud));

				int numGoodK2Points = 0;
				for (int k2y = 0; k2y < K2_PCL_HEIGHT; k2y += k2Steps)
				{
					for (int k2x = 0; k2x < K2_PCL_WIDTH; k2x += k2Steps)
					{
						int index = k2y*K2_PCL_WIDTH + k2x;

						bool isGood = !isinf(k2TmpCloud[index].x);
						if (isGood)
						{
							pclData->k2Cloud.positions[numGoodK2Points] = index;
							pclData->k2Cloud.points[numGoodK2Points] = k2TmpCloud[index];
							numGoodK2Points++;
						}

					}
				}

				pclData->k2Cloud.numGoodPoints = numGoodK2Points;
			}
		}
	}


	//REALSENSE
	if (rsIsInit && rsIsStarted)
	{
		
		rs->update();
		rsDepthTex.loadData(rs->getDepth8uFrame());

		if (memoryIsConnected)
		{
			vector<ofVec3f> pc = rs->getPointCloud();
			vector<int> ind = rs->getIndices();
			int numGoodRSPoints = 0;
			int numRSPoints = pc.size();
			for (int rsi = 0; rsi < NUM_RS_PIXELS && rsi < numRSPoints; rsi+=rsSteps)
			{
				pclData->rsCloud.points[numGoodRSPoints] = pc.at(rsi) / 1000.;
				pclData->rsCloud.positions[numGoodRSPoints] = ind.at(rsi);
				numGoodRSPoints++;
			}

			pclData->rsCloud.numGoodPoints = numGoodRSPoints;
			
		}
				
	}

	
}

//--------------------------------------------------------------
void ofApp::draw(){

	ofClear(ofColor::black);
	ofSetColor(ofColor::white);


	int imgSize = ofGetWidth()/3;

	
	for (int i = 0; i < NUM_KINECTS1; i++)
	{
		ofxKinect * k = k1List[i];

		bool kIsConnected = ofxKinect::isDeviceConnected(i) && k->isConnected();
		if (kIsConnected)
		{
			float tx = i % 2;
			float ty = floor(i / 2);
			k->drawDepth(ofRectangle(tx*imgSize, ty*imgSize, imgSize, imgSize));
		} else
		{
			//ofLogNotice("Kinect " + ofToString(i) + " is not connected, not drawing.");
		}
	}
	

	
	if (k2.isOpen() && k2.isFrameNew())
	{
		
		k2Depthtex.draw(imgSize * 2, 0, imgSize, imgSize);
	} else
	{
		//ofLogNotice("Kinect2 is not open or no new frame");
	}


	if (rsIsInit && rsIsStarted)
	{
		rsDepthTex.draw(imgSize, imgSize, imgSize, imgSize);
	}

	
	ofDrawBitmapStringHighlight("FPS : " + ofToString(ofGetFrameRate()), 10, 10);
	ofDrawBitmapStringHighlight("Kinect 1 Steps : " + ofToString(k1Steps), 10, 20);
	ofDrawBitmapStringHighlight("Kinect 2 Steps : " + ofToString(k2Steps), 10, 40);
	ofDrawBitmapStringHighlight("RealSense Steps : " + ofToString(rsSteps), 10, 60);
}

void ofApp::exit()
{

	for (int i = 0; i < NUM_KINECTS1; i++)
	{
		bool kIsConnected = ofxKinect::isDeviceConnected(i) && k1List[i]->isConnected();
		if (kIsConnected)
		{
			k1List[i]->close();
		}

		delete k1List[i];
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

void ofApp::processOSC()
{
	// check for waiting messages
	while (oscReceiver.hasWaitingMessages()) {
		// get the next message
		ofxOscMessage m;
		oscReceiver.getNextMessage(m);
		if (m.getAddress() == "/kinect1/steps") {
			k1Steps = max(2,m.getArgAsInt(0));
		}else if (m.getAddress() == "/kinect2/steps") {
			k2Steps = max(4, m.getArgAsInt(0));
		}else if (m.getAddress() == "/realsense/steps") {
			rsSteps = max(4, m.getArgAsInt(0));
		}

	}
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
