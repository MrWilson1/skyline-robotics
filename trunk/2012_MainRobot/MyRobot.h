/**
 * MyRobot.h
 * 
 * This is the main class for the robot.
 * 
 * Skyline Spartabots, Team 2976
 * Made for 2012 Robot Rumble
 */

#include "WPILib.h"
#include "range.h"
#include "controller.h"

class MainRobot : public SimpleRobot
{
private:
	static const double kMotorWait = 0.005;		// In seconds
	static const double kWatchdogExpiration = 0.1;	// In seconds
	
protected:
	RobotDrive *mRobotDrive;
	AnalogChannel *mUltrasoundSensor;
	Jaguar *mMotorTestJaguar;
	Servo *mTopServo;
	Servo *mBottomServo;
	
	Joystick *mLeftJoystick;
	Joystick *mRightJoystick;
	Joystick *mMotorTestJoystick;
	Kinect *mKinect;
	
	Distance *mDistance;
	static const int kControllerLen = 1;
	BaseController *mControllers[kControllerLen];

public:
	MainRobot(void);
	void InitializeHardware(void);
	void InitializeControllers(void);
	void InitializeSoftware(void);
	void Autonomous(void);
	void OperatorControl(void);
};
