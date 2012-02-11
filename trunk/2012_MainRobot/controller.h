/**
 * controller.h
 * 
 * This file contains all the code used to allow a human
 * to control any aspect of the robot.
 * 
 * Every class in here should have 'BaseComponent' as
 * their parent class, so they can be placed under
 * MyRobot::mComponentCollection.
 * 
 * Skyline Spartabots, Team 2976
 * Made for 2012 Robot Rumble
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

// System libraries
#include <utility.h>
#include <math.h>

// 3rd-party libraries
#include "WPILib.h"

// Program libraries
#include "component.h"
#include "tools.h"

/**
 * TankJoysticks
 * 
 * Takes two joysticks and drives the robot, tank-style.
 * 
 * Todo: document what every button on the Joystick does.
 */
class TankJoysticks : public BaseController
{
protected:
	Joystick *mLeftJoystick;
	Joystick *mRightJoystick;
	RobotDrive *mRobotDrive;
	
	static const float kSpeedFactorMin = 0.3;
	static const float kSpeedFactorMax = 1.0;
	
public:
	TankJoysticks(RobotDrive *, Joystick *, Joystick *);
	void Run(void);

protected:
	float GetSpeedDecreaseFactor(void);
};

class SingleJoystick : public BaseController
{
protected:
	RobotDrive *mRobotDrive;
	Joystick *mJoystick;
	
	typedef std::pair<float, float> Wheel;
	
public:
	SingleJoystick(RobotDrive *, Joystick *);
	void Run(void);
	void GetDiagnostics(void);;
	
};

/**
 * KinectController
 * 
 * Takes a Kinect and uses hand gestures to drive the robot.
 * 
 * Todo: Document what hand gestures this uses.
 */
class KinectController : public BaseController
{
protected:
	RobotDrive *mRobotDrive;
	Kinect *mKinect;
	static const float kArmMinZ = 0;
	static const float kArmMaxZ = 0.38;
	static const float kShootThresholdY = 0.2;
	
public:
	KinectController(RobotDrive *, Kinect *);
	void Run(void);
	
protected:
	void HaltRobot(void);
	bool IsPlayerReady(void);
	bool IsPlayerShooting(void);
	float GetLeftArmDistance(void);
	float GetRightArmDistance(void);
};

float Round(float, int);

#endif
