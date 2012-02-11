/**
 * sensors.h
 * 
 * Classes that uses sensors (in a complex manner) and returns data.
 * 
 * Skyline Spartabots, Team 2976
 * Made for 2012 Robot Rumble
 */

#ifndef SENSORS_H_
#define SENSORS_H_

// 3rd party libraries
#include "WPILib.h"

// Program modules
#include "component.h"

class RangeFinder
{
protected:
	AnalogChannel *mUltrasoundSensor;
	static const INT32 kWallDistanceMin = 140;	// In inches.
	static const INT32 kWallDistanceMax = 200;	// In inches.
	
public:
	RangeFinder(AnalogChannel *);
	float FromWallInches(void);
	INT32 FromWallRaw(void);
	bool IsInShootingRange(void);
};

class RangeFinderTest : public BaseController
{
protected:
	RangeFinder *mRangeFinder;
	
public:
	RangeFinderTest(RangeFinder *);
	void Run(void);
};

class GyroTest : public BaseController
{
protected:
	Gyro *mGyro;
	
public:
	GyroTest(Gyro *);
	void Run(void);
};

#endif
