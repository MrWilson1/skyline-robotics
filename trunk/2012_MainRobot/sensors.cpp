/**
 * sensors.cpp
 * 
 * Classes that uses sensors (in a complex manner) and returns data.
 * 
 * Skyline Spartabots, Team 2976
 * Made for 2012 Robot Rumble
 */

#include "sensors.h"

RangeFinder::RangeFinder(AnalogChannel *ultrasoundSensor)
{
	mUltrasoundSensor = ultrasoundSensor;
	return;
}

/**
 * RangeFinder::FromWallInches
 * 
 * Finds the distance from the wall in inches.
 * Note that this can at times be inaccurate.
 * 
 * Input:
 *   - None
 * 
 * Output:
 *   - The distance from the wall to the ultrasound
 *     sensor in inches.
 * 
 * Side-effects:
 *   - None
 */
float RangeFinder::FromWallInches(void)
{
	INT32 rawDistance = FromWallRaw();
	float inchesDistance = (float) rawDistance * 0.5;		// Based on experimental data.
	return inchesDistance;
}

/**
 * RangeFinder::FromWallRaw
 * 
 * Finds the raw distance from the wall.
 * Use RangeFinder::FromWallInches over this one.
 * 
 * Input:
 *   - None
 * 
 * Output:
 *   - The raw distance returned by the ultrasonic sensor
 * 
 * Side-effects:
 *   - None
 */
INT32 RangeFinder::FromWallRaw(void)
{
	return mUltrasoundSensor->GetAverageValue();
}

/**
 * RangeFinder::IsInShootingRange(void)
 * 
 * Input:
 *   - None
 * 
 * Output:
 *   - Returns true if the robot is within a certain distance from the wall.
 *     (...which means this should return true whenever the robot is in the key)
 *  
 * Side-effects:
 *   - None 
 */
bool RangeFinder::IsInShootingRange(void)
{
	float inchesDistance = FromWallInches();
	bool isInRange = false;
	if ((kWallDistanceMin <= inchesDistance) and (inchesDistance <= kWallDistanceMax)) {
		isInRange = true;
	}
	return isInRange;
}

/**
 * GyroTest::GyroTest(Gyro *)
 * 
 * Tests the gyro
 * 
 * Inputs:
 *   - Gyro *gyro
 *     A pointer to a gyro object
 * 
 * Outputs:
 *   - None
 * 
 * Side-effects:
 *   - Creates the object
 */
GyroTest::GyroTest(Gyro *gyro)
{
	mGyro = gyro;
	mGyro->Reset();
}

/**
 * GyroTest::Run
 * 
 * Side-effects:
 *   - Prints the value of the gyro to SmartDashboard
 */
void GyroTest::Run()
{
	SmartDashboard::GetInstance()->Log(mGyro->GetAngle(), "Gyro");
}
