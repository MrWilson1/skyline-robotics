/**
 * @file PrototypeRobot.h
 * 
 * @brief Definitions for the prototype robot.
 * 
 * @details To simplify deployment, each robot should be 
 * encapsulated in a single class.
 * 
 * This class is obliged to provide two public methods --
 * PrototypeRobot::Autonomous() and PrototypeRobot::OperatorControl.
 * 
 * This class is also responsible for instantiating 
 * every possible class that will be used in this program.
 * The instantiated objects are then passed as pointers to
 * any classes that need them.
 */

#ifndef PROTOTYPEROBOT_H_
#define PROROTYPEROBOT_H_

// System libraries
#include <vector>

// 3rd party libraries
#include "WPILib.h"

// Program modules
#include "arm.h"
#include "sensors.h"
#include "driving.h"
#include "components.h"

/**
 * @brief This class bundles together everything to ultimately
 * run and control the robot.
 * 
 * @details
 * When the robot starts, a single instance of this class is 
 * created by a macro (provided by WPILib).  This class
 * must provide several methods in order to be fully
 * functional:
 *   - Autonomous()
 *   - OperatorControl()
 * 
 * It may also provide additional methods provided by
 * WPILib and SimpleRobot.
 * 
 * This class is the only one that is allowed to
 * create instances of any WPILib classes, Components,
 * and Controllers (to prevent accidental memory leaks
 * with pointers).  However, any class is allowed to
 * create instances of normal classes.
 */
class PrototypeRobot : public SimpleRobot
{
private:
	// Safety constants
	static const double kMotorWait = 0.01;		// In seconds
	static const double kWatchdogExpiration = 1;	// In seconds
	
protected:
	// Hardware
	RobotDrive *mRobotDrive;
	
	SpeedController *mLeftFrontDrive;
	SpeedController *mLeftBackDrive;
	SpeedController *mRightFrontDrive;
	SpeedController *mRightBackDrive;
	
	Compressor *mCompressor;
	Solenoid *mSolenoid1;
	Solenoid *mSolenoid2;
	
	// Input devices
	Joystick *mLeftJoystick;
	Joystick *mRightJoystick;
	Joystick *mTwistJoystick;
	Kinect *mKinect;
	
	// Components
	KinectStick *mLeftKinectStick;
	KinectStick *mRightKinectStick;
	TargetFinder *mTargetFinder;
	BaseArmComponent *mArm;
	
	PneumaticArm *mPneumaticArm;
	
	// Controller -- see controller.h
	vector<BaseController*> mControllerCollection;

public:
	PrototypeRobot();
	~PrototypeRobot();
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

