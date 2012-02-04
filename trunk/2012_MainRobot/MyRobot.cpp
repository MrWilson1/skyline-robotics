/**
 * MyRobot.h
 * 
 * This is the main class for the robot.
 * 
 * Skyline Spartabots, Team 2976
 * Made for 2012 Robot Rumble
 */

#include "MyRobot.h"
#define SdLog SmartDashboard::GetInstance()->Log

MainRobot::MainRobot(void)
{
	SdLog("Starting", "State");
	InitializeHardware();
	SdLog("Hardware initialized", "State");
	InitializeControllers();
	SdLog("Controllers Initialized", "State");
	InitializeSoftware();
	SdLog("Software Initialized", "State");
	GetWatchdog().SetExpiration(kWatchdogExpiration);
	SdLog("Finished Initialization", "State");
	return;
}

void MainRobot::InitializeHardware(void)
{
	mRobotDrive = new RobotDrive(1, 2, 3, 4);
	// AnalogChannel(UINT8 moduleNumber, UINT32 channel)
	mUltrasoundSensor = new AnalogChannel(1, 1); 
	mMotorTestJaguar = new Jaguar(5);
	mTopServo = new Servo(7);
	mBottomServo = new Servo(8);
	return;
}

void MainRobot::InitializeControllers(void)
{
	mLeftJoystick = new Joystick(1);
	mRightJoystick = new Joystick(2);
	mMotorTestJoystick = new Joystick(3);
	mKinect = Kinect::GetInstance();
}	

void MainRobot::InitializeSoftware(void)
{
	mDistance = new Distance(mUltrasoundSensor);
	
	mControllers[0] = new TankJoysticks(mRobotDrive, mLeftJoystick, mRightJoystick);
	//mControllers[0] = new KinectController(mRobotDrive, mKinect);
	
	
	
	//mControllers[1] = new MotorTestController(mRobotDrive, mMotorTestJoystick, mMotorTestJaguar);
	//mControllers[2] = new ServoTestController(mRobotDrive, mTopServo, mBottomServo, mRightJoystick, mLeftJoystick);
	return;
}

void MainRobot::Autonomous(void)
{
	GetWatchdog().SetEnabled(true);
	while (IsAutonomous()) {
		GetWatchdog().Feed();
		Wait(kMotorWait);
	}
}

void MainRobot::OperatorControl(void) 
{
	GetWatchdog().SetEnabled(true);
	SmartDashboard::GetInstance()->Log("Operator Control", "State");
	
	while (IsOperatorControl())
	{
		for(int i=0; i<kControllerLen; i++) {
			mControllers[i]->Run();
			GetWatchdog().Feed();
		}
		SdLog(mDistance->FromWallInches(), "Ultrasound");
		SmartDashboard::GetInstance()->Log("Still alive", "MainRobot::OperatorControl loop");
		GetWatchdog().Feed();
		Wait(kMotorWait);
	}
	return;
}


START_ROBOT_CLASS(MainRobot);

