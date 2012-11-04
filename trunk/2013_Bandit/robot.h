#ifndef ROBOT_H_
#define ROBOT_H_

#include "WPILib.h"
#include "component.h"
#include "ports.h"
#include "drive.h"

class MainRobot : public SimpleRobot {
public:
	void OperatorControl();
	void RobotInit(void);
	void Autonomous();
	
protected:
	Joystick *mRightJoystick;
	Joystick *mLeftJoystick;
	Joystick *mTwistJoystick;
	
	SpeedController *mRearLeftMotor;
	SpeedController *mRearRightMotor;
	SpeedController *mFrontLeftMotor;
	SpeedController *mFrontRightMotor;
	
	map<string, Component *>mComponents;
	Drive *mDrive;
	RobotDrive *mRobotDrive;
	/*Shooter *mShooter;
	Arm *mArm;
	Controller *mController;	*/
};

#endif
