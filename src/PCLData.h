#pragma once

/*
struct Color
{
	float r;
	float g;
	float b;
	float a;
};
*/

/*
struct PCLPoint
{
	Vec3 pos;
	Color color;
};
*/

struct RSCloud
{
	int numGoodPoints;
	ofVec3f pclCenter;
	ofVec3f points[NUM_RS_PIXELS];
	int goodPointIndices[NUM_RS_PIXELS];

};

struct K1Cloud
{
	int numGoodPoints;
	ofVec3f pclCenter;
	ofVec3f points[NUM_K1_PIXELS];
	int goodPointIndices[NUM_K1_PIXELS];
	int quadIndices[NUM_K1_PIXELS * 4];
	int numQuads;
};

struct K2Cloud
{
	int numGoodPoints;
	ofVec3f pclCenter;
	int numBodiesTracked;

	ofVec3f headPos;
	ofVec3f leftHandPos;
	ofVec3f rightHandPos;
	ofVec3f neckPos;
	ofVec3f torsoPos;
	
	ofVec3f points[NUM_K2_PIXELS];
	int goodPointIndices[NUM_K2_PIXELS];
	int quadIndices[NUM_K2_PIXELS * 4];
	int numQuads;
};



struct PCLData {
	bool isReady;
	K1Cloud k1Clouds[NUM_KINECTS1];
	K2Cloud k2Cloud;
	RSCloud rsCloud;
};

