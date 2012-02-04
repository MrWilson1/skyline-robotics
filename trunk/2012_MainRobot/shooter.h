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
#include "sensors.h"

class Shooter : public BaseComponent 
{
protected:
	SpeedController *mSpeedController1;
	SpeedController *mSpeedController2;
	Joystick *mJoystick;
	RangeFinder *mRangeFinder;
	static const int kMaxSpeed = 96;
	static const int kMinSpeed = 12;
	
public:
	Shooter(SpeedController*, SpeedController*, Joystick*, RangeFinder*);
	void Shoot();
	float CalculateSpeed(float);
	void LoadBall();
};

#endif
