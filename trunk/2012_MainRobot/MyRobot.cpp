

#include "MyRobot.h"


MainRobot::MainRobot(void)
{
}

/**
 * @brief Initializes all hardware, input devices, and software 
 * for the entire robot.
 * 
 * @details This class will also enable the watchdog.
 */
void MainRobot::RobotInit(void)
{
	InitializeHardware();
	InitializeInputDevices();
	InitializeComponents();
	InitializeControllers();
	
	//mTargetTask = Task(
	//		"TargetFinder",
	//		mTargetFinder->Run,
	//		Task::kDefaultPriority);
	
	GetWatchdog().SetExpiration(kWatchdogExpiration);
	return;
}

/**
 * @brief Initializes any code provided by WPILib meant to
 * interface directly with specific hardware components.
 * 
 * @details Currently initializes the 
 *   - Robot drive
 *   - Jaguars for the shooter
 *   - Ultrasound sensor
 *   - Gyro 
 */
void MainRobot::InitializeHardware(void)
{
	mRobotDrive = new RobotDrive(
			Ports::Pwm1,		// Left front 
			Ports::Pwm2, 		// Left back
			Ports::Pwm3, 		// Right front
			Ports::Pwm4);		// Right back
	
	mTopLeftShooter = new Jaguar(
			Ports::Pwm5);
	mTopRightShooter = new Jaguar(
			Ports::Pwm6);
	mBottomLeftShooter = new Jaguar(
			Ports::Pwm7);
	mBottomRightShooter = new Jaguar(
			Ports::Pwm8);
	
	mUltrasoundSensor = new AnalogChannel(
			Ports::Module1,
			Ports::AnalogChannel1);
	mGyro = new Gyro(
			Ports::Module1,
			Ports::AnalogChannel2);
	// The camera is technically a hardware component, but WPILib's
	// AxisCamera class has a built-in static method for returning
	// instances of a camera
	return;
}

/**
 * @brief Initializes any hardware used by the laptop to send
 * data over to the robot (joysticks, etc).
 * 
 * @details Currently initializes the
 *   - Joysticks
 *   - Kinect
 */
void MainRobot::InitializeInputDevices(void)
{
	mLeftJoystick = new Joystick(
			Ports::Usb1);
	mRightJoystick = new Joystick(
			Ports::Usb2);
	mTwistJoystick = new Joystick(
			Ports::Usb3);
	mKinect = Kinect::GetInstance();
}

/**
 * @brief Initializes any software components that 
 * bundles together hardware needed to provide 
 * additional functionality.
 * 
 * @details For example, code to run the shooter would
 * be initialized here because it uses several
 * Jaguars and a Joystick to function.
 * 
 * Ideally, each class instantiated should have BaseComponent
 * as a parent class.
 */
void MainRobot::InitializeComponents(void)
{
	mRangeFinder = new RangeFinder(mUltrasoundSensor);
	mShooter = new Shooter(
			mTopLeftShooter,
			mTopRightShooter,
			mBottomLeftShooter,
			mBottomRightShooter,
			mRangeFinder);
}

/**
 * @brief Initializes any software that takes in user input to 
 * manipulate hardware or software, or needs to be called
 * periodically during operator control
 * 
 * @details Each object initialized in this file must inherit 
 * BaseController.  They are assigned to the heap, and appended 
 * to the end of MainRobot::mControllerCollection, 
 */
void MainRobot::InitializeControllers(void)
{
	//mControllerCollection.push_back(new TankJoysticks(mRobotDrive, mLeftJoystick, mRightJoystick));
	//mControllerCollection.push_back(new ShooterController(mShooter, mRightJoystick));
	//mControllerCollection.push_back(new RangeFinderTest(mRangeFinder));
	//mControllerCollection.push_back(new GyroTest(mGyro));
	//mControllerCollection.push_back(new TargetFinder());
	
	mControllerCollection.push_back(new ShooterController(mShooter, mTwistJoystick));
	//mControllerCollection.push_back(new KinectController(mRobotDrive, mKinect));
	mControllerCollection.push_back(new SingleJoystick(mRobotDrive, mTwistJoystick));
	// TODO: change the below to use the new vector collection.
	//mControllers[0] = new KinectController(mRobotDrive, mKinect);
	//mControllers[1] = new MotorTestController(mRobotDrive, mMotorTestJoystick, mMotorTestJaguar);
	//mControllers[2] = new ServoTestController(mRobotDrive, mTopServo, mBottomServo, mRightJoystick, mLeftJoystick);
	return;
}

/**
 * MainRobot::~MainRobot
 *
 * Destroys all hardware, input devices, and software
 * created for the entire robot.  I believe this should
 * be automatically handled, but I just wanted to
 * make sure.
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
MainRobot::~MainRobot(void)
{
	delete mRobotDrive;
	//delete mElevatorSpeedController;
	delete mUltrasoundSensor;
	delete mGyro;
	
	delete mLeftJoystick;
	delete mRightJoystick;
	delete mTwistJoystick;
	
	delete mRangeFinder;
	int collectionSize = (int) mControllerCollection.size();
	for (int i=0; i<collectionSize; i++) {
		delete mControllerCollection.at(i);
	}
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


START_ROBOT_CLASS(MainRobot);

