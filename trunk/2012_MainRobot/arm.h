/*
 * @file arm.h
 * 
 * @brief Provides code to control the arm.
 */

#ifndef ARM_H_
#define ARM_H_

#include "WPILib.h"
#include "motorLimitWatchdog.h"
#include "components.h"

class Arm
{
public:
	static const float kMotorDirection = 1.0;
	static const float kMotorSpeed = 1.0;
protected:
	SpeedController *mArmMotor;
	DigitalInput *mTopLimit;
	DigitalInput *mBottomLimit;
	MotorLimitWatchdog *mMotorWatchdog;
public:
	Arm (SpeedController *, DigitalInput *, DigitalInput *);
	~Arm ();
	
	void GoUp(void);
	void GoDown(void);
	void Stop(void);
};

/**
 * @brief Interacts with joystick to control the arm.
 */

class ArmController : public BaseController
{
protected:
	Arm *mArm;
	Joystick *mJoystick;
public:
	ArmController (Arm *, Joystick *);
	void Run(void);
};

#endif
