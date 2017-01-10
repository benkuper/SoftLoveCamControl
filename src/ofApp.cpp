#pragma warning(push)
#pragma warning(disable:4244 4838 4996 4267)
#include "ofApp.h"
#pragma warning(pop)
//--------------------------------------------------------------
void ofApp::setup(){

	ofSetWindowShape(1200,800);
	ofSetFrameRate(60);

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
		bool initResult = k->init(false,true,true);
		k->open(i);
		ofLogNotice("Setup", "Opening Kinect %i, connected / deviceConnected ? %i / %i", i, k->isConnected(),ofxKinect::isDeviceConnected(i));
		k1List.push_back(k);
		ofSleepMillis(100);
		k1Senders[i].init("K1-" + ofToString(i), K1_PCL_WIDTH, K1_PCL_HEIGHT);
		
	}

	//Kinect 2
	k2.open();
	ofLogNotice("Setup", "KinectV2 is Open ? %i", k2.isOpen());
	if (k2.isOpen())
	{
		
		k2.initDepthSource();
		k2Mapper = k2.getDepthSource()->getCoordinateMapper();
		k2.initBodySource();
		k2.initColorSource();
		k2.getColorSource();
		k2.initBodyIndexSource();
		numBodiesTracked = 0;

		k2DepthTex.allocate(K2_PCL_WIDTH, K2_PCL_HEIGHT, OF_IMAGE_COLOR);
		k2BodyTex.allocate(K2_PCL_WIDTH, K2_PCL_HEIGHT, OF_IMAGE_COLOR);

		//k2ColorTex.allocate(K2_COLOR_WIDTH, K2_COLOR_HEIGHT, OF_IMAGE_COLOR);
		k2ColorImage.allocate(K2_PCL_WIDTH, K2_PCL_HEIGHT, OF_IMAGE_COLOR);
		k2Sender.init("SoftLoveK2", K2_PCL_WIDTH, K2_PCL_HEIGHT);
	}

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
	
	for (int i = 0; i < NUM_KINECTS1; i++) freezeK1[i] = false;
	freezeK2 = false;
	freezeRS = false;

	doDraw = true;
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

	if (memoryIsConnected)
	{
		pclData->isReady = false;
		mtx.lock();
	}

	//KINECT 1
	for (int i = 0; i < NUM_KINECTS1; i++)
	{
		ofxKinect * k = k1List[i];
		if (!k->isConnected()) continue;

		k->update();
		
		if (memoryIsConnected && !freezeK1[i])
		{
			int numQuads = 0;
			int numGoodK1Points = 0;
			int tIndex = 0;
			ofVec3f pclCenter;

			for (int ty = 0; ty < K1_PCL_HEIGHT; ty+=k1Steps)
			{
				for (int tx = 0; tx < K1_PCL_WIDTH; tx+= k1Steps)
				{
					int index = ty * K1_PCL_WIDTH + tx;
					
					ofVec3f p = k->getWorldCoordinateAt(tx*2, ty*2)*ofVec3f(1,-1,1)/1000.; //ofVec3f(tx*1./K1_PCL_WIDTH,ty*1./K1_PCL_HEIGHT,.5f);// 
		
					bool isGood = p.x != 0 && !isnan(p.x);
					pclData->k1Clouds[i].points[index] = p;

					if (isGood)
					{
						pclData->k1Clouds[i].goodPointIndices[numGoodK1Points] = index;
						pclCenter += p;
						numGoodK1Points++;


						int index2 = ty*K1_PCL_WIDTH + tx + k1Steps;
						int index3 = (ty + k1Steps)*K1_PCL_WIDTH + tx + k1Steps;
						int index4 = (ty + k1Steps)*K1_PCL_WIDTH + tx;

						if (isK1PointGood(i,index2) && isK1PointGood(i,index3) && isK1PointGood(i,index4))
						{
							pclData->k1Clouds[i].quadIndices[numQuads * 4] = index;
							pclData->k1Clouds[i].quadIndices[numQuads * 4 + 1] = index2;
							pclData->k1Clouds[i].quadIndices[numQuads * 4 + 2] = index3;
							pclData->k1Clouds[i].quadIndices[numQuads * 4 + 3] = index4;
							numQuads++;
						}
					}				
				}
			}

			k1Senders[i].send(k->getTexture());

			pclData->k1Clouds[i].numGoodPoints = numGoodK1Points;
			pclData->k1Clouds[i].pclCenter = pclCenter / numGoodK1Points;
			pclData->k1Clouds[i].numQuads = numQuads;
		}
		
		
	}


	// KINECT 2
	if (k2.isOpen())
	{
		k2.update();
		if (k2.isFrameNew() && !freezeK2)
		{
			k2DepthTex = k2.getDepthSource()->getTexture();
			k2BodyTex = k2.getBodyIndexSource()->getTexture(); 

			if (memoryIsConnected)
			{
				k2BodyPixels = k2.getBodyIndexSource()->getPixels();
				int newBodiesTracked = 0;
				auto& bodies = k2.getBodySource()->getBodies();
			
			
				for (auto& body : bodies) {
					if (body.tracked) {
						newBodiesTracked++;
					
						auto &atlas = body.getBonesAtlas();
						pclData->k2Cloud.headPos = body.joints.at(JointType::JointType_Head).getPosition();
						pclData->k2Cloud.leftHandPos = body.joints.at(JointType::JointType_HandLeft).getPosition();
						pclData->k2Cloud.rightHandPos = body.joints.at(JointType::JointType_HandRight).getPosition();
						pclData->k2Cloud.neckPos = body.joints.at(JointType::JointType_Neck).getPosition();
						pclData->k2Cloud.torsoPos = body.joints.at(JointType::JointType_SpineBase).getPosition();
						break;
					}
				}

				if (newBodiesTracked != numBodiesTracked)
				{
					numBodiesTracked = newBodiesTracked;
					ofLog() << "Num bodies tracked " << numBodiesTracked;
				}

				
				k2Mapper->MapDepthFrameToCameraSpace(NUM_K2_PIXELS,k2.getDepthSource()->getPixels().begin() , NUM_K2_PIXELS, reinterpret_cast<CameraSpacePoint*>(pclData->k2Cloud.points));
				
				
				auto &colorPixels = k2.getColorSource()->getPixels();				
				//ofLogNotice(ofToString(colorPixels.getWidth()) + " / " + ofToString(colorPixels.getHeight()));

				k2Mapper->MapDepthFrameToColorSpace(NUM_K2_PIXELS, k2.getDepthSource()->getPixels().begin(), NUM_K2_PIXELS, colorIndices);

				int numBodyPoints = 0;
				int numGoodK2Points = 0;
				ofVec3f pclCenter;

				int numQuads = 0;
				
				
				if (colorPixels.size() != 0)
				{
					for (int k2y = 0; k2y < K2_PCL_HEIGHT; k2y++)
					{
						for (int k2x = 0; k2x < K2_PCL_WIDTH; k2x++)
						{
							
							int index = k2y*K2_PCL_WIDTH + k2x;
							ofVec2f mappedCoord(floor(colorIndices[index].X), floor(colorIndices[index].Y));
							if (mappedCoord.x > 0 && mappedCoord.y > 0 && mappedCoord.x < K2_COLOR_WIDTH && mappedCoord.y < K2_COLOR_HEIGHT)
							{
								//ofLogNotice(ofToString(mappedCoord.x) + " / " + ofToString(mappedCoord.y));
								k2ColorImage.setColor(k2x, k2y, colorPixels.getColor(mappedCoord.x, mappedCoord.y));
							}
							

							if (k2x % k2Steps != 0 || k2y % k2Steps != 0) continue;


							bool isGood = isK2PointGood(index);

							if (isGood)
							{
								int index2 = k2y*K2_PCL_WIDTH + k2x + k2Steps;
								int index3 = (k2y + k2Steps)*K2_PCL_WIDTH + k2x + k2Steps;
								int index4 = (k2y + k2Steps)*K2_PCL_WIDTH + k2x;

								if (isK2PointGood(index2) && isK2PointGood(index3) && isK2PointGood(index4))
								{
									pclData->k2Cloud.quadIndices[numQuads * 4] = index;
									pclData->k2Cloud.quadIndices[numQuads * 4 + 1] = index2;
									pclData->k2Cloud.quadIndices[numQuads * 4 + 2] = index3;
									pclData->k2Cloud.quadIndices[numQuads * 4 + 3] = index4;
									numQuads++;
								}

								pclData->k2Cloud.goodPointIndices[numGoodK2Points] = index;
								pclCenter += pclData->k2Cloud.points[index];
								numGoodK2Points++;


							} else
							{
								pclData->k2Cloud.points[index] = ofVec3f();
							}

						}
					}

					k2ColorImage.update();
					//k2ColorTex = k2.getColorSource()->getTexture();			
					//if(k2ColorTex.getWidth() > 0 && k2ColorTex.getHeight() > 0)
					k2Sender.send(k2ColorImage.getTexture());

					pclData->k2Cloud.numGoodPoints = numGoodK2Points;
					pclData->k2Cloud.pclCenter = pclCenter / numGoodK2Points;
					pclData->k2Cloud.numBodiesTracked = numBodiesTracked;
					pclData->k2Cloud.numQuads = numQuads;
				}

			}
		}
	}


	//REALSENSE
	if (rsIsInit && rsIsStarted)
	{
		
		rs->update();
		rsDepthTex.loadData(rs->getDepth8uFrame());

		if (memoryIsConnected && !freezeRS)
		{
			vector<ofVec3f> pc = rs->getPointCloud();
			//vector<int> ind = rs->getIndices();
			ofVec3f pclCenter;

			int numGoodRSPoints = 0;
			int numRSPoints = pc.size();
			for (int rsi = 0; rsi < NUM_RS_PIXELS && rsi < numRSPoints; rsi+=rsSteps)
			{
				pclData->rsCloud.points[numGoodRSPoints] = pc.at(rsi) / 1000.;
				pclCenter += pclData->rsCloud.points[numGoodRSPoints];
				pclData->rsCloud.goodPointIndices[numGoodRSPoints] = rsi;//rsi.at(rsi);
				numGoodRSPoints++;
			}
			pclData->rsCloud.pclCenter = pclCenter / numGoodRSPoints;
			pclData->rsCloud.numGoodPoints = numGoodRSPoints;
			
		}
				
	}

	if (memoryIsConnected)
	{
		pclData->isReady = true;
		mtx.unlock();
	}
	
}


bool ofApp::isK1PointGood(int kinectIndex, int pIndex)
{
	return  pclData->k1Clouds[kinectIndex].points[pIndex].x != 0 && !isnan(pclData->k1Clouds[kinectIndex].points[pIndex].x);;
}

bool ofApp::isK2PointGood(int pIndex)
{
	bool result = !isinf(pclData->k2Cloud.points[pIndex].x);
	if (result && numBodiesTracked > 0)
	{
		result = k2BodyPixels[pIndex] < 255;
	}

	return result;
}

//--------------------------------------------------------------
void ofApp::draw(){

	ofClear(ofColor::black);
	ofSetColor(ofColor::white);


	int imgSize = ofGetWidth()/3;

	
	if (doDraw)
	{
		for (int i = 0; i < NUM_KINECTS1; i++)
		{
			ofPushStyle();
			ofSetColor(ofColor::white);
			ofxKinect * k = k1List[i];

			bool kIsConnected = ofxKinect::isDeviceConnected(i) && k->isConnected();
			float tx = i % 2;
			float ty = floor(i / 2);

			if (kIsConnected)
			{
				k->draw(ofRectangle(tx*imgSize, ty*imgSize, imgSize, imgSize));
				ofSetColor(ofColor::green, 100);
				k->drawDepth(ofRectangle(tx*imgSize, ty*imgSize, imgSize, imgSize));
				
			} else
			{
				ofSetColor(ofColor::purple, 100);
				ofDrawRectangle(tx*imgSize, ty*imgSize, imgSize, imgSize);
			}

			if (freezeK1[i])
			{
				ofSetColor(ofColor::red, 100);
				ofDrawRectangle(tx*imgSize, ty*imgSize, imgSize, imgSize);
			}
			ofPopStyle();

		}
		
		if (k2.isOpen())
		{
			
			ofPushStyle();

			ofSetColor(ofColor::white);
			k2ColorImage.draw(imgSize * 2, 0, imgSize, imgSize);

			/*
			ofSetColor(ofColor::white, 30);
			k2DepthTex.draw(imgSize * 2, 0, imgSize, imgSize);
			*/

			if (numBodiesTracked > 0)
			{
				ofSetColor(ofColor::red, 50);
				k2BodyTex.draw(imgSize * 2, 0, imgSize, imgSize);
			}

			
			if (freezeK2)
			{
				ofSetColor(ofColor::red, 100);
				ofDrawRectangle(imgSize * 2, 0, imgSize, imgSize);
			}

			ofPopStyle();

		}
		

		if (rsIsInit && rsIsStarted)
		{
			rsDepthTex.draw(imgSize, imgSize, imgSize, imgSize);
		} else
		{
			ofSetColor(ofColor::purple, 100);
			ofDrawRectangle(imgSize, imgSize, imgSize, imgSize);
		}

		if (freezeRS)
		{
			ofSetColor(ofColor::red, 100);
			ofDrawRectangle(imgSize, imgSize, imgSize, imgSize);
		}

	}
	
	ofDrawBitmapStringHighlight("FPS : " + ofToString(ofGetFrameRate()), 10, 10);
	ofDrawBitmapStringHighlight("Kinect 1 Steps : " + ofToString(k1Steps), 10, 30);
	ofDrawBitmapStringHighlight("Kinect 2 Steps : " + ofToString(k2Steps), 10, 50);
	ofDrawBitmapStringHighlight("RealSense Steps : " + ofToString(rsSteps), 10, 70);
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
	switch (key)
	{
	case '1':
		if (NUM_KINECTS1 >= 1) freezeK1[0] = !freezeK1[0];
		
		break;

	case '2':
		if (NUM_KINECTS1 >= 2) freezeK1[1] = !freezeK1[1];
		break;

	case '3':
		freezeK2 = !freezeK2;
		break;

	case '4':
		if (NUM_KINECTS1 >= 3) freezeK1[2] = !freezeK1[2];
		break;

	case '5':
		freezeRS = !freezeRS;
		break;

	case '6':
		if (NUM_KINECTS1 >= 4) freezeK1[3] = !freezeK1[3];
		break;

	case 'd':
		doDraw = !doDraw;
		break;

	case '+':
		k1Steps++;
		break;
	case '-':
		k1Steps = max(k1Steps - 1, 1);
		break;

	case '*':
		k2Steps++;
		break;
	case '/':
		k2Steps = max(k2Steps - 1, 1);
		break;
	}
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
