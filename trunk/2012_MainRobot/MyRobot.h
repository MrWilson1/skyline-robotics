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

// System libraries
#include <vector>

// 3rd party libraries
#include "WPILib.h"

// Program modules
#include "sensors.h"
#include "controller.h"
#include "component.h"

class MainRobot : public SimpleRobot
{
private:
	// Safety constants
	static const double kMotorWait = 0.01;		// In seconds
	static const double kWatchdogExpiration = 0.1;	// In seconds
	
protected:
	// Hardware
	RobotDrive *mRobotDrive;
	AnalogChannel *mUltrasoundSensor;	// For ultrasound
	Gyro *mGyro;
	SpeedController *mElevatorSpeedController;
	
	// Input devices
	Joystick *mLeftJoystick;
	Joystick *mRightJoystick;
	Joystick *mTwistJoystick;
	Kinect *mKinect;
	
	// Software
	RangeFinder *mRangeFinder;
	GyroTest *mGyroTest;
	
	// Controller -- see controller.h
	std::vector<BaseComponent*> mComponentCollection;

public:
	MainRobot();
	~MainRobot();
	void Autonomous();
	void OperatorControl();
	
protected:
	void InitializeHardware();
	void InitializeInputDevices();
	void InitializeSoftware();
};

#endif

