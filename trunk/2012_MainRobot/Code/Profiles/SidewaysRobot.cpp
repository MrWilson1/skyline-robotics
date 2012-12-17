#include "SidewaysRobot.h"

SidewaysRobot::SidewaysRobot(void) :
	SimpleRobot()
{
	InitializeHardware();
	InitializeInputDevices();
	InitializeComponents();
	InitializeControllers();
}

void SidewaysRobot::RobotInit(void)
{
	GetWatchdog().SetExpiration(kWatchdogExpiration);
	return;
}

void SidewaysRobot::InitializeHardware(void)
{
	mLeftFrontDrive = new Jaguar(
			Ports::Crio::Module1,
			Ports::DigitalSidecar::Pwm1);
	mLeftBackDrive = new Jaguar(
			Ports::Crio::Module1,
			Ports::DigitalSidecar::Pwm2);
	mRightFrontDrive = new Jaguar(
			Ports::Crio::Module1,
			Ports::DigitalSidecar::Pwm3);
	mRightBackDrive = new Jaguar(
			Ports::Crio::Module1,
			Ports::DigitalSidecar::Pwm4);
	
	mRobotDrive = new RobotDrive(
			mLeftFrontDrive,
			mLeftBackDrive,
			mRightFrontDrive,
			mRightBackDrive);
	//mRobotDrive->SetInvertedMotor(mRobotDrive->kFrontLeftMotor, true);
	//mRobotDrive->SetInvertedMotor(mRobotDrive->kRearLeftMotor, true);
	//mRobotDrive->SetInvertedMotor(mRobotDrive->kFrontRightMotor, true);
	//mRobotDrive->SetInvertedMotor(mRobotDrive->kRearRightMotor, true);
	return;
}

void SidewaysRobot::InitializeInputDevices(void)
{
	mLeftJoystick = new Joystick(
			Ports::Usb2);
	mRightJoystick = new Joystick(
			Ports::Usb3);
	
	mTwistJoystick = new Joystick(
			Ports::Usb4);
	
	//mXboxController = new XboxController(
	//		Ports::Usb1);
	
}

void SidewaysRobot::InitializeComponents(void)
{
	
}

void SidewaysRobot::InitializeControllers(void)
{
	vector<BaseController *> controllers;
	controllers.push_back(new TankJoysticks(
			mRobotDrive, 
			mLeftJoystick, 
			mRightJoystick));
	controllers.push_back(new XboxDrive(
			mRobotDrive, 
			mXboxController));
	controllers.push_back(new XboxDriveSingle(
			mRobotDrive,
			mXboxController));
	mControllerCollection.push_back(new ControllerSwitcher(controllers));
	
	return;
}

SidewaysRobot::~SidewaysRobot(void)
{
	
}

void SidewaysRobot::Autonomous(void)
{
	GetWatchdog().SetEnabled(true);
	while (IsAutonomous()) {
		GetWatchdog().Feed();
		Wait(kMotorWait);
	}
}

void SidewaysRobot::OperatorControl(void) 
{
	GetWatchdog().SetEnabled(true);
	
	int collectionSize = (int) mControllerCollection.size();
	while (IsOperatorControl())
	{
		for(int i=0; i<collectionSize; i++) {
			mControllerCollection.at(i)->Run();
			GetWatchdog().Feed();
			Wait(kMotorWait);
		}
	}
	return;
}

