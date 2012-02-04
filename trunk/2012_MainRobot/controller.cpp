/**
 * controller.cpp
 * 
 * Various implementation of controllers.
 * 
 * Skyline Spartabots, Team 2976
 * Made for 2012 Robot Rumble
 * 
 * Test B
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
	mRobotDrive->TankDrive(leftY, rightY);
	
	SmartDashboard::GetInstance()->Log(leftY, "left_y");
	SmartDashboard::GetInstance()->Log(rightY, "right_y");
	return;
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


int Prettify(float input, int power) 
{
	int output = (int) (input * pow(10, power));
	return output;
}

int Prettify(float input)
{
	return Prettify(input, 3);
}
