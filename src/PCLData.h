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
	ofVec3f points[NUM_RS_PIXELS];
	int positions[NUM_RS_PIXELS];

};

struct K1Cloud
{
	int numGoodPoints;
	ofVec3f points[NUM_K1_PIXELS];
	int positions[NUM_K1_PIXELS];
};

struct K2Cloud
{
	int numGoodPoints;
	ofVec3f points[NUM_K2_PIXELS];
	int positions[NUM_K2_PIXELS];
};



struct PCLData {
	K1Cloud k1Clouds[NUM_KINECTS1];
	K2Cloud k2Cloud;
	RSCloud rsCloud;
};

