/**
 * controller.h
 * 
 * Various implementation of controllers.
 * 
 * Skyline Spartabots, Team 2976
 * Made for 2012 Robot Rumble
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

// 3rd-party libraries
#include "WPILib.h"

// Program libraries
#include "component.h"

/**
 * TankJoysticks
 * 
 * Takes two joysticks and drives the robot, tank-style
 */
class TankJoysticks : public BaseComponent
{
protected:
	Joystick *mLeftJoystick;
	Joystick *mRightJoystick;
	RobotDrive *mRobotDrive;
	
public:
	TankJoysticks(RobotDrive *, Joystick *, Joystick *);
	void Run(void);
};

/**
 * KinectController
 * 
 * Takes a Kinect and uses hand gestures to drive the robot.
 */
class KinectController : public BaseComponent
{
protected:
	RobotDrive *mRobotDrive;
	Kinect *mKinect;
	
public:
	KinectController(RobotDrive *, Kinect *);
	void Run(void);
	void HaltRobot(void);
	float GetLeftArmDistance(void);
	float GetRightArmDistance(void);
	float Coerce(float, float, float, float, float);
};

/*
class MotorTestController : public BaseController
{
protected:
	Joystick *mJoystick;
	SpeedController *mSpeedController;
	
public:
	MotorTestController(RobotDrive *, Joystick *, SpeedController *);
	void Run(void);
};

class ServoTestController : public BaseController
{
protected:
	Joystick *mTopJoystick, *mBottomJoystick;
	Servo *mTopServo, *mBottomServo;
	
	
public:
	ServoTestController(RobotDrive *, Servo *, Servo *, Joystick *, Joystick *);
	void Run(void);
};
*/
#endif
