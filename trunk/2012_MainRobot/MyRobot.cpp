/**
 * MyRobot.h
 * 
 * This is the main class for the robot.  It bundles together all
 * the software and hardware, and is the main entry point to
 * the robot.
 * 
 * Skyline Spartabots, Team 2976
 * Made for 2012 Robot Rumble
 */

#include "MyRobot.h"
#include "component.h"

/**
 * MainRobot::MainRobot
 * 
 * Initializes all hardware, input devices, and software 
 * for the entire robot.
 * 
 * Inputs:
 *   - None
 * 
 * Outputs:
 *   - None
 * 
 * Side effects:
 *   - See description
 *   - Enables the watchdog
 */
MainRobot::MainRobot(void)
{
	InitializeHardware();
	InitializeInputDevices();
	InitializeSoftware();
	GetWatchdog().SetExpiration(kWatchdogExpiration);
	return;
}

/**
 * MainRobot::InitializeHardware
 * 
 * Initializes any code provided by WPILib meant to
 * interface directly with specific hardware components.
 * 
 * Inputs:
 *   - None
 * 
 * Outputs:
 *   - None
 * 
 * Side effects:
 *   - See description
 */
void MainRobot::InitializeHardware(void)
{
	mRobotDrive = new RobotDrive(
			Ports::Pwm1,		// Left front 
			Ports::Pwm2, 		// Left back
			Ports::Pwm3, 		// Right front
			Ports::Pwm4);		// Right back
	
	mUltrasoundSensor = new AnalogChannel(
			Ports::Module1,
			Ports::AnalogChannel1); 
	mMotorTestJaguar = new Jaguar(
			Ports::Pwm5);
	mTopServo = new Servo(
			Ports::DigitalIo1);
	mBottomServo = new Servo(
			Ports::DigitalIo2);
	return;
}

/**
 * MainRobot::InitializeInputDevices
 * 
 * Initializes any hardware used by the laptop to send
 * data over to the robot (joysticks, etc).
 * 
 * Inputs:
 *   - None
 * 
 * Outputs:
 *   - None
 * 
 * Side-effects:
 *   - See description
 * 
 * Notes:
 *   - Also gets an instance of the Kinect.
 */
void MainRobot::InitializeInputDevices(void)
{
	mLeftJoystick = new Joystick(
			Ports::Usb1);
	mRightJoystick = new Joystick(
			Ports::Usb2);
	mKinect = Kinect::GetInstance();
}	

/**
 * MainRobot::InitializeSoftware
 * 
 * Initializes any software components that 
 * bundle together input devices or hardware
 * code that provided related functionality.
 * 
 * For example, code to run the shooter would
 * be initialized here because it uses several
 * Jaguars and a Joystick to function.
 * 
 * Inputs:
 *   - None
 * 
 * Outputs:
 *   - None
 * 
 * Side-effects:
 *   - See description
 */
void MainRobot::InitializeSoftware(void)
{
	mRangeFinder = new RangeFinder(mUltrasoundSensor);
	//mRangeFinderTest = new RangeFinderTest(mRangeFinder);
	
	mControllers[0] = new TankJoysticks(mRobotDrive, mLeftJoystick, mRightJoystick);
	//mControllers[0] = new KinectController(mRobotDrive, mKinect);
	//mControllers[1] = new MotorTestController(mRobotDrive, mMotorTestJoystick, mMotorTestJaguar);
	//mControllers[2] = new ServoTestController(mRobotDrive, mTopServo, mBottomServo, mRightJoystick, mLeftJoystick);
	return;
}

/**
 * MainRobot::Autonomous
 * 
 * This is a mandatory function required for the robot to function.
 * When the 'Autonomous' toggle is selected and enabled on the 
 * FRC Dashboard, this function will run.
 * 
 * It is meant to be run once, at the start of the match, during
 * Hybrid mode.
 * 
 * Inputs:
 *   - None
 * 
 * Outputs:
 *   - None
 * 
 * Side-effects:
 *   - See description
 */
void MainRobot::Autonomous(void)
{
	GetWatchdog().SetEnabled(true);
	while (IsAutonomous()) {
		GetWatchdog().Feed();
		Wait(kMotorWait);
	}
}

/**
 * MainRobot::OperatorControl
 * 
 * This is a mandatory function required for the robot to function.
 * When the 'Autonomous' toggle is selected and enabled on the 
 * FRC Dashboard, this function will run.
 * 
 * It currently uses the two joysticks to drive (tank drive).
 * Specifically, it takes any object that's been added to
 * 'mControllers' and calls the 'run' function on it.
 * 
 * Any object that wants to be run during Operator Control
 * should inherit either 'BaseComponent' or 
 * 'BaseController' and be added to mControllers.
 * 
 * Inputs:
 *   - None
 * 
 * Outputs:
 *   - None
 * 
 * Side-effects:
 *   - See description 
 */
void MainRobot::OperatorControl(void) 
{
	GetWatchdog().SetEnabled(true);
	SmartDashboard::GetInstance()->Log("Operator Control", "State");
	
	while (IsOperatorControl())
	{
		for(int i=0; i<kControllerLen; i++) {
			mControllers[i]->Run();
			GetWatchdog().Feed();
			Wait(kMotorWait);
		}
	}
	return;
}


START_ROBOT_CLASS(MainRobot);

