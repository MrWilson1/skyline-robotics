/**
 * range.h
 * 
 * Uses ultrasound and other sensors to report information on the robot's distance from a object,
 * or its location.
 * 
 * Skyline Spartabots, Team 2976
 * Made for 2012 Robot Rumble
 */

#include "WPILib.h"

class Distance
{
private:
	AnalogChannel *mUltrasoundSensor;
	static const INT32 kWallDistanceMin = 140;	// In inches.
	static const INT32 kWallDistanceMax = 200;	// In inches.
	
public:
	Distance(AnalogChannel *);
	float FromWallInches(void);
	INT32 FromWallRaw(void);
	bool IsInShootingRange(void);
};
