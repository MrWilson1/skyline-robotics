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
#include "components.h"

class Shooter
{
protected:
	SpeedController *mTopLeftSpeedController;
	SpeedController *mTopRightSpeedController;
	SpeedController *mBottomLeftSpeedController;
	SpeedController *mBottomRightSpeedController;
	RangeFinder *mRangeFinder;
	static const float kShooterAngle = 45;
	static const float kShooterHeight = 50;
	static const float kBasketHeight = 98;
	static const float kGravity = 386.4; // inches per second per second
	static const float kMaxSpeed = 28.373;
	static const float kMinSpeed = 22.698; // assuming that the minimum distance is 5 feet, and the maximum distance is 12 feet
	static const float kReductionFactor = 0.9;

public:
    Shooter(SpeedController*, SpeedController*, SpeedController*, SpeedController*, RangeFinder*);
    void SetSpeedManually(float);
    void SetSpeedAutomatically();
    float CalculateSpeed(float);
	float CalculateDistance();
	void Run();
};

class ShooterController : public BaseController
{
protected:
	Shooter *mShooter;
	Joystick *mJoystick;
public:
	ShooterController(Shooter *, Joystick *);
	void Run(void);
	
};

#endif
