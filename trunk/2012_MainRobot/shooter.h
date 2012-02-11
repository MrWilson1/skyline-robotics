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
	static const int kMaxDistance = 144;
	static const int kMinDistance = 12;
	static const float kShooterAngle = 45;
	static const float kShooterHeight = 50;
	static const float kBasketHeight = 98;
	static const float gravity = 386.4; // inches per second per second
	static const float kArenaXMaximum = 648;
	static const float kArenaYMaximum = 324;

public:
    Shooter(SpeedController*, SpeedController*, Joystick*, RangeFinder*);
    void Shoot();
    float CalculateSpeed(float);
    void LoadBall();
	float CalculateDistance();
	void Run();
};

#endif
