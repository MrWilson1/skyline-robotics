/**
 * controller.h
 * 
 * Various implementation of controllers.
 * 
 * Skyline Spartabots, Team 2976
 * Made for 2012 Robot Rumble
 */

#include "WPILib.h"

class BaseController
{
protected:
	RobotDrive *mRobotDrive;
	
public:
	BaseController(RobotDrive *);
	virtual void Run(void);
};

class TankJoysticks : public BaseController
{
protected:
	Joystick *mLeftJoystick;
	Joystick *mRightJoystick;
	
public:
	TankJoysticks(RobotDrive *, Joystick *, Joystick *);
	void Run(void);
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
class KinectController : public BaseController
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


int Prettify(float, int);
int Prettify(float);
