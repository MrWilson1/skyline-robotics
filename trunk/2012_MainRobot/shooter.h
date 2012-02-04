/**
 * shooter.h
 * 
 * Operates device that shoots basketballs by spinning wheels.
 * 
 * Skyline Spartabots, Team 2976
 * Made for 2012 Robot Rumble
 */

#ifndef SHOOTER_H_
#define SHOOTER_H_

#include "WPILib.h"

class Shooter : public BaseComponent 
{
protected:
	SpeedController *mSpeedController1;
	SpeedController *mSpeedController2;
	Joystick *mJoystick;
	RangeFinder *mRangeFinder;
	
public:
	Shooter(SpeedController*, SpeedController*, Joystick*, RangeFinder*);
	void Shoot();
	float CalculateSpeed(float);
};

#endif
