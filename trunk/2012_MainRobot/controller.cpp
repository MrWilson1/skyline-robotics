/**
 * controller.cpp
 * 
 * Various implementation of controllers.
 * 
 * Skyline Spartabots, Team 2976
 * Made for 2012 Robot Rumble
 */

#include <math.h>
#include "controller.h"

TankJoysticks::TankJoysticks(RobotDrive *robotDrive, Joystick *leftJoystick, Joystick *rightJoystick)
{
	mRobotDrive = robotDrive;
	mLeftJoystick = leftJoystick;
	mRightJoystick = rightJoystick;
	return;
}

void TankJoysticks::Run(void)
{
	float leftY = mLeftJoystick->GetY();
	float rightY = mRightJoystick->GetY();
	
	float speedFactor = GetSpeedDecreaseFactor(); 
	
	leftY *= speedFactor;
	rightY *= speedFactor;
	
	mRobotDrive->TankDrive(leftY, rightY);
	
	SmartDashboard::GetInstance()->Log(leftY, "left_y");
	SmartDashboard::GetInstance()->Log(rightY, "right_y");
	return;
}

/**
 * TankController::GetSpeedDecreaseFactor
 * 
 * Reads the throttle on the left joystick to determine the
 * overall robot speed.  You multiply the value returned by
 * this function to the actual raw speed to slow the overall
 * power down based on user input.
 * 
 * The higher the throttle, the more the
 * power.  There's no way to choke the speed all the way
 * down to zero for usability reasons -- the lowest number this
 * will return is about 0.3, and the highest is exactly 1.0 to
 * prevent the robot from totally halting.
 * 
 * Inputs:
 *   - None
 * 
 * Outputs:
 *   - A float, ranging from about 0.3 to 1.0
 * 
 * Side-effects:
 *   - None
 */
float TankJoysticks::GetSpeedDecreaseFactor(void)
{
	float rawFactor = mLeftJoystick->GetThrottle();	// Returns a range from 0.0 to 1.0
	float range = kSpeedFactorMax - kSpeedFactorMin;
	float speedFactor = rawFactor * range + kSpeedFactorMin;  // This relies on the rawFactor being from 0.0 to 1.0.
	return speedFactor;
}


/*
MotorTestController::MotorTestController(RobotDrive *robotdrive, Joystick *joystick, SpeedController *speedController):
		BaseController(robotdrive)
{
	mJoystick = joystick;
	mSpeedController = speedController;
	return;
}


ServoTestController::ServoTestController(RobotDrive *robotDrive, Servo *topServo, Servo *bottomServo, 
		Joystick *topJoystick, Joystick *bottomJoystick):
		BaseController(robotDrive)
{
	mTopServo = topServo;
	mBottomServo = bottomServo;
	mTopJoystick = topJoystick;
	mBottomJoystick = bottomJoystick;
}

void ServoTestController::Run(void)
{
	mTopServo->Set((mTopJoystick->GetThrottle() + 1.0) * 0.5);
	mBottomServo->Set((mBottomJoystick->GetThrottle() + 1.0) * 0.5);
	return;
}
*/




KinectController::KinectController(RobotDrive *robotDrive, Kinect *kinect)
{
	mRobotDrive = robotDrive;
	mKinect = kinect;
}

void KinectController::Run(void)
{
	SmartDashboard::GetInstance()->Log("Apparently alive", "Kinect Code");
	if (mKinect->GetTrackingState() == Kinect::kTracked) {
		SmartDashboard::GetInstance()->Log("Tracked", "Kinect State");
		mRobotDrive->TankDrive(GetLeftArmDistance(), GetRightArmDistance());
	} else {
		SmartDashboard::GetInstance()->Log("Not tracked", "Kinect State");
		HaltRobot();
	}
}

void KinectController::HaltRobot(void)
{
	mRobotDrive->TankDrive(0.0, 0.0);
}

float KinectController::GetLeftArmDistance(void)
{
	float originJoint = mKinect->GetSkeleton().GetShoulderLeft().z;
	float movingJoint = mKinect->GetSkeleton().GetWristLeft().z;
	
	float distance = originJoint - movingJoint;
	float output = Coerce(distance, 0, 0.45, -1, 1);
	
	SmartDashboard::GetInstance()->Log(output, "Left movement");
	
	return output;
}

float KinectController::GetRightArmDistance(void)
{
	float originJoint = mKinect->GetSkeleton().GetShoulderRight().z;
	float movingJoint = mKinect->GetSkeleton().GetWristRight().z;
	
	float distance = originJoint - movingJoint;
	float output = Coerce(distance, 0, 0.45, -1, 1);
	SmartDashboard::GetInstance()->Log(output, "Right movement");
	return output;	
}

float KinectController::Coerce(float number, float rawMin, float rawMax, float adjustedMin, float adjustedMax)
{
	// Check inputs for validity.
	if (rawMin >= rawMax) {
		HaltRobot();
		SmartDashboard::GetInstance()->Log("Input rawMin greater then rawMax.", "ERROR @ KinectController::Coerce");
	}
	if (adjustedMin >= adjustedMax) {
		HaltRobot();
		SmartDashboard::GetInstance()->Log("Input adjustedMin greater then adjustedMax", "ERROR @ KinectController::Coerce");
	}
	
	if (number < rawMin) {
		number = rawMin;
	} else if (number > rawMax) {
		number = rawMax;
	}
	
	float percentage = (number - rawMin) / (rawMax - rawMin);
	return percentage * (adjustedMax - adjustedMin) - 1; 
}


/**
 * SingleJoystick::SingleJoystick
 * 
 * This is a form of controlling the robot by using the single
 * Extreme 3D Pro joystick.  This differs from our other
 * joysticks because it comes with twisting motions.
 */
SingleJoystick::SingleJoystick(RobotDrive *robotDrive, Joystick *joystick)
{
	mRobotDrive = robotDrive;
	mJoystick = joystick;
}


void SingleJoystick::Run()
{
	float rawX = mJoystick->GetX();
	float rawY = mJoystick->GetY();
	float rawZ = mJoystick->GetZ();
	float rawTwist = mJoystick->GetTwist();
	float rawThrottle = mJoystick->GetThrottle();
	bool isTriggerOn = mJoystick->GetTrigger();
	
	SmartDashboard::GetInstance()->Log(rawX, "SingleJoystick::rawX");
	SmartDashboard::GetInstance()->Log(rawY, "SingleJoystick::rawY");
	SmartDashboard::GetInstance()->Log(rawZ, "SingleJoystick::rawZ");
	SmartDashboard::GetInstance()->Log(rawTwist, "SingleJoystick::rawTwist");
	SmartDashboard::GetInstance()->Log(rawThrottle, "SingleJoystick::rawThrottle");
	SmartDashboard::GetInstance()->Log(isTriggerOn, "SingleJoystick::isTriggerOn");
}



int Prettify(float input, int power) 
{
	int output = (int) (input * pow(10, power));
	return output;
}

int Prettify(float input)
{
	return Prettify(input, 3);
}
