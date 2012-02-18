/**
 * @brief Contains the code used to control the robot
 * based on user input.
 * 
 * @details
 * Every class in here should have 'BaseController' as
 * their parent class, so they can be placed under
 * MyRobot::mComponentCollection.
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

// System libraries
#include <utility.h>
#include <math.h>

// 3rd-party libraries
#include "WPILib.h"

// Our code
#include "components.h"

/**
 * @brief A baseclass for any controller that uses 
 * a joystick.
 * 
 * @details
 * Implements some common functions.
 */
class BaseJoystickController : public BaseController
{
protected:
	static const float kSpeedFactorMin = 0.3;
	static const float kSpeedFactorMax = 1.0;
	float squareInput(float);
	float getSpeedFactor(Joystick *);
	
public:
	BaseJoystickController();
	void Run() = 0;
};


/**
 * @brief Takes two joysticks and drives the robot, tank-style.
 * 
 * @details
 * Todo: document what every button on the Joystick does.
 */
class TankJoysticks : public BaseJoystickController
{
protected:
	Joystick *mLeftJoystick;
	Joystick *mRightJoystick;
	RobotDrive *mRobotDrive;
	
	static const float kSpeedFactorMin = 0.3;
	static const float kSpeedFactorMax = 1.0;
	
public:
	TankJoysticks(RobotDrive *, Joystick *, Joystick *);
	void Run(void);

protected:
	float GetSpeedDecreaseFactor(void);
};

/**
 * @brief Takes a single joystick and drives the robot,
 * arcade style.
 * 
 * @details
 * Todo: document what every button on the Joystick
 * does.
 */
class SingleJoystick : public BaseJoystickController
{
protected:
	RobotDrive *mRobotDrive;
	Joystick *mJoystick;
		
public:
	SingleJoystick(RobotDrive *, Joystick *);
	float GetSpeedDecreaseFactor();
	void Run(void);
};

/**
 * @brief Takes a Kinect and uses hand gestures to drive the robot.
 * 
 * @details
 * This particular KinectController uses the z-axis of the hands
 * to set the speeds of each side of the robot (tank-style)
 * 
 * Todo: Document specifically what hand gestures this uses.
 */
class KinectController : public BaseController
{
protected:
	RobotDrive *mRobotDrive;
	Kinect *mKinect;
	static const float kArmMinZ = 0;
	static const float kArmMaxZ = 0.38;
	static const float kShootThresholdY = 0.2;
	
public:
	KinectController(RobotDrive *, Kinect *);
	void Run(void);
	
protected:
	void HaltRobot(void);
	bool IsPlayerReady(void);
	bool IsPlayerShooting(void);
	float GetLeftArmDistance(void);
	float GetRightArmDistance(void);
};

#endif
