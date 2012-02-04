/**
 * range.h
 * 
 * Uses ultrasound and other sensors to report information on the robot's distance from a object,
 * or its location.
 * 
 * Skyline Spartabots, Team 2976
 * Made for 2012 Robot Rumble
 */

#include "range.h"

Distance::Distance(AnalogChannel *ultrasoundSensor)
{
	Distance::mUltrasoundSensor = ultrasoundSensor;
	return;
}

float Distance::FromWallInches(void)
{
	INT32 rawDistance = Distance::FromWallRaw();
	float inchesDistance = (float) rawDistance * 0.5;		// Based on experimental data.
	return inchesDistance;
}

INT32 Distance::FromWallRaw(void)
{
	return mUltrasoundSensor->GetAverageValue();
}

bool Distance::IsInShootingRange(void)
{
	float inchesDistance = Distance::FromWallInches();
	bool isInRange = false;
	if ((kWallDistanceMin <= inchesDistance) and (inchesDistance <= kWallDistanceMax)) {
		isInRange = true;
	}
	return isInRange;
}

