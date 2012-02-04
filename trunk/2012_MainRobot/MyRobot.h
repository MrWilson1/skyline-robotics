/**
 * MyRobot.h
 * 
 * This is the main class for the robot.  It is the only class allowed to 
 * initialize any hardware components, and is responsible for bundling
 * together everything.
 * 
 * Skyline Spartabots, Team 2976
 * Made for 2012 Robot Rumble
 */

#ifndef MYROBOT_H_
#define MYROBOT_H_

// 3rd party libraries
#include "WPILib.h"

// Program modules
#include "sensors.h"
#include "controller.h"

class MainRobot : public SimpleRobot
{
private:
	// Safety constants
	static const double kMotorWait = 0.005;		// In seconds
	static const double kWatchdogExpiration = 0.1;	// In seconds
	
protected:
	// Hardware
	RobotDrive *mRobotDrive;
	AnalogChannel *mUltrasoundSensor;	// For ultrasound
	Jaguar *mMotorTestJaguar;
	Servo *mTopServo;
	Servo *mBottomServo;
	
	// Input devices
	Joystick *mLeftJoystick;
	Joystick *mRightJoystick;
	Joystick *mMotorTestJoystick;
	Kinect *mKinect;
	
	// Software
	RangeFinder *mRangeFinder;
	RangeFinderTest *mRangeFinderTest;
	
	// Controller -- see controller.h
	static const int kControllerLen = 1;
	BaseController *mControllers[kControllerLen];

public:
	MainRobot(void);
	void Autonomous(void);
	void OperatorControl(void);
	
protected:
	void InitializeHardware(void);
	void InitializeInputDevices(void);
	void InitializeSoftware(void);
};

#endif
