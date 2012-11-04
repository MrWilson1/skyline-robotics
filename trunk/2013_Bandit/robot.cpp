#include "robot.h"

void MainRobot::RobotInit(void) {
	//hardware
	mRightJoystick = new Joystick(Ports::Usb1);
	mLeftJoystick = new Joystick(Ports::Usb2);
	mTwistJoystick = new Joystick(Ports::Usb3);
	
	mRearLeftMotor = new Jaguar(Ports::DigitalSidecar::Pwm1);
	mRearRightMotor = new Jaguar(Ports::DigitalSidecar::Pwm2);
	mFrontLeftMotor = new Jaguar(Ports::DigitalSidecar::Pwm3);
	mFrontRightMotor = new Jaguar(Ports::DigitalSidecar::Pwm4);
	
	//software
	mRobotDrive = new RobotDrive(mFrontLeftMotor, mRearLeftMotor, mFrontRightMotor, mRearRightMotor);
	mDrive = new Drive(mRobotDrive);
	
	map<string, Component *>mComponents;
	mComponents.insert(pair<string, Component *>("Drive", mDrive));
	
	return;
}
