/**
 * @brief The main entry point of the robot.
 * 
 * @details FIRST has provided a macro that takes the name of 
 * this class and runs it once deployed on the cRIO (see
 * the bottom of this file).
 * 
 * This class is obliged to provide two public methods --
 * MainRobot::Autonomous() and MainRobot::OperatorControl.
 * 
 * This class is also responsible for instantiating 
 * every possible class that will be used in this program.
 * The instantiated objects are then passed as pointers to
 * any classes that need them.
 * 
 * @author Michael Lee
 * 
 * @license	GNU Lesser GPL
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
#include "shooter.h"
#include "target.h"

class MainRobot : public SimpleRobot
{
private:
	// Safety constants
	static const double kMotorWait = 0.01;		// In seconds
	static const double kWatchdogExpiration = 1;	// In seconds
	
protected:
	// Hardware
	RobotDrive *mRobotDrive;
	AnalogChannel *mUltrasoundSensor;	// For ultrasound
	Gyro *mGyro;
	SpeedController *mElevatorSpeedController;
	SpeedController *mTopLeftShooter;
	SpeedController *mTopRightShooter;
	SpeedController *mBottomLeftShooter;
	SpeedController *mBottomRightShooter;
	
	// Input devices
	Joystick *mLeftJoystick;
	Joystick *mRightJoystick;
	Joystick *mTwistJoystick;
	Kinect *mKinect;
	
	// Components
	RangeFinder *mRangeFinder;
	Shooter *mShooter;
	
	// Controller -- see controller.h
	std::vector<BaseController*> mControllerCollection;
	//TargetFinder mTargetFinder;
	//Task mTargetTask;

public:
	MainRobot();
	~MainRobot();
	void RobotInit();
	void Autonomous();
	void OperatorControl();
	//void Disabled();
	
protected:
	void InitializeHardware();
	void InitializeInputDevices();
	void InitializeComponents();
	void InitializeControllers();
};

#endif

