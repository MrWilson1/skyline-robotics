/**
 * controller.cpp
 * 
 * This file contains all the code used to allow a human
 * to control any aspect of the robot.
 * 
 * Every class in here should have 'BaseComponent' as
 * their parent class, so they can be placed under
 * MyRobot::mComponentCollection.
 * 
 * Skyline Spartabots, Team 2976
 * Made for 2012 Robot Rumble
 */

#include "controller.h"

/**
 * TankJoysticks::TankJoysticks
 * 
 * Creates an instance of a controller.  It requires the
 * robot drive, and two joysticks.
 * 
 * Inputs:
 *   - RobotDrive *robotDrive
 *     A pointer to the RobotDrive object created in MyRobot::InitiateHardware
 *   - Joystick *leftJoystick
 *     A pointer to the left joystick (controls the left side of the robot)
 *   - Joystick *rightJoystick
 *     A pointer to the right joystick (controls the right side of the robot)
 * 
 * Outputs:
 *   - None
 * 
 * Side-effects:
 *   - See description
 */
TankJoysticks::TankJoysticks(RobotDrive *robotDrive, Joystick *leftJoystick, Joystick *rightJoystick)
{
	mRobotDrive = robotDrive;
	mLeftJoystick = leftJoystick;
	mRightJoystick = rightJoystick;
}

/**
 * TankJoysticks::Run
 * 
 * This method is called automatically during MyRobot::OperatorControl
 * It should contain no 'Wait' statements or excessive loops.
 * 
 * Inputs:
 *   - None
 * 
 * Outputs:
 *   - None
 * 
 * Side-effects:
 *   - Drives the robot
 *   - Logs the left and right speed values to the SmartDashboard
 *   - Reads values from both joysticks 
 */
void TankJoysticks::Run(void)
{
	SmartDashboard::GetInstance()->Log("Alive", "Tank status");
	float leftY = mLeftJoystick->GetY();
	float rightY = mRightJoystick->GetY(); 
	
	float squaredLeftY = (leftY < 0) ? -(leftY * leftY) : (leftY * leftY);
	float squaredRightY = (rightY < 0) ? -(rightY * rightY) : (rightY * rightY);
	
	float speedFactor = GetSpeedDecreaseFactor();
	
	squaredLeftY *= speedFactor;
	squaredRightY *= speedFactor;
	
	SmartDashboard::GetInstance()->Log(squaredLeftY, "left_y");
	SmartDashboard::GetInstance()->Log(squaredRightY, "right_y");
	
	mRobotDrive->TankDrive(squaredLeftY, squaredRightY);
	
	
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
 *   - Logs data to SmartDashboard
 *   - Reads data from the joystick
 */
float TankJoysticks::GetSpeedDecreaseFactor(void)
{
	float rawFactor = mLeftJoystick->GetThrottle();	// Returns a range from -1.0 to 1.0
	float normalizedFactor = (rawFactor + 1) / 2; 	// Sets the range from between 0.0 to 1.0

	float range = kSpeedFactorMax - kSpeedFactorMin;
	float speedFactor = normalizedFactor * range + kSpeedFactorMin;  // This relies on the rawFactor being from 0.0 to 1.0.
	
	// Something peculiar is happening with the abs functions:
	// std::abs seems to do integers only, and I find it 
	// suspicious that fabs isn't in the std namespace.
	// Doing it manually just in case.
	float absSpeedFactor = (speedFactor < 0) ? -speedFactor : speedFactor;
	
	return absSpeedFactor;
}


///////////////////////


/**
 * SingleJoystick::SingleJoystick
 * 
 * This is a form of controlling the robot by using the single
 * Extreme 3D Pro joystick.  This differs from our other
 * joysticks because it comes with twisting motions.
 * 
 * Inputs:
 *   - RobotDrive *robotDrive
 *     A pointer to the robotDrive
 *   - Joystick *joystick
 *     A pointer to the joystick -- must be the Extreme 3D Pro
 *     joystick
 * 
 * Output:
 *   - None
 * 
 * Side-effects:
 *   - Creates the object
 */
SingleJoystick::SingleJoystick(RobotDrive *robotDrive, Joystick *joystick)
{
	mRobotDrive = robotDrive;
	mJoystick = joystick;
}

/**
 * SingleJoystick::Run()
 * 
 * In progress, currently empty.
 */
void SingleJoystick::Run()
{
	//float magnitude = GetMagnitude();
	//float magnitudeMultiplier = GetMagnitudeMultiplier();
	
	
}

/**
 * SingleJoystick::GetDiagnostics()
 * 
 * Get diagnostic data.
 * 
 * Inputs:
 *   - None
 * 
 * Outputs:
 *   - None
 *   
 * Side-effects:
 *   - Logs all the joystick values to SmartDashboard
 */
void SingleJoystick::GetDiagnostics()
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




// Todo: Document KinectController
KinectController::KinectController(RobotDrive *robotDrive, Kinect *kinect)
{
	mRobotDrive = robotDrive;
	mKinect = kinect;
}

void KinectController::Run(void)
{
	if (mKinect->GetTrackingState() == Kinect::kTracked) {
		SmartDashboard::GetInstance()->Log("Tracked", "Kinect State");
		if (IsPlayerReady()) {
			SmartDashboard::GetInstance()->Log("Apparently alive", "Kinect Code");
			mRobotDrive->TankDrive(GetLeftArmDistance(), GetRightArmDistance());
		} else {
			SmartDashboard::GetInstance()->Log("Frozen", "Kinect State");
			HaltRobot();
		}
	} else {
		SmartDashboard::GetInstance()->Log("Not tracked", "Kinect State");
		HaltRobot();
	}
	
	if (IsPlayerShooting()) {
		SmartDashboard::GetInstance()->Log("Shooting", "Player shooting");
	} else {
		SmartDashboard::GetInstance()->Log(" ", "Player shooting");
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
	float output = Tools::Coerce(distance, kArmMinZ, kArmMaxZ, -1, 1);
	
	SmartDashboard::GetInstance()->Log(Round(output, 2), "Left movement");
	
	return output;
}

float KinectController::GetRightArmDistance(void)
{
	float originJoint = mKinect->GetSkeleton().GetShoulderRight().z;
	float movingJoint = mKinect->GetSkeleton().GetWristRight().z;
	
	float distance = originJoint - movingJoint;
	float output = Tools::Coerce(distance, kArmMinZ, kArmMaxZ, -1, 1);
	
	SmartDashboard::GetInstance()->Log(Round(output, 2), "Right movement");
	
	return output;	
}

bool KinectController::IsPlayerReady(void)
{
	float rightOrigin = mKinect->GetSkeleton().GetShoulderRight().x;
	float rightMoving = mKinect->GetSkeleton().GetWristRight().x;
	
	float leftOrigin = mKinect->GetSkeleton().GetShoulderLeft().x;
	float leftMoving = mKinect->GetSkeleton().GetWristLeft().x;
	
	if ((rightOrigin > rightMoving) or (leftOrigin < leftMoving)) {
		return false;
	}
	return true;
}

bool KinectController::IsPlayerShooting(void)
{
	float rightOrigin = mKinect->GetSkeleton().GetShoulderRight().y;
	float rightMoving = mKinect->GetSkeleton().GetWristRight().y;
	
	float leftOrigin = mKinect->GetSkeleton().GetShoulderLeft().y;
	float leftMoving = mKinect->GetSkeleton().GetWristLeft().y;
	
	float rightDelta = rightMoving - rightOrigin;
	float leftDelta = leftMoving - leftOrigin;
	
	// Like shooting a basketball.
	if ((rightDelta > kShootThresholdY) or (leftDelta > kShootThresholdY)) {
		return true;
	} else {
		return false;
	}
}

/**
 * Round
 * 
 * A helper function to round numbers to a given
 * precision.
 * 
 * Todo: Remove this function.
 */
float Round(float input, int precision)
{
	float multiple = pow(10, precision);
	SmartDashboard::GetInstance()->Log(multiple, "Multiple");
	return floorf(input * multiple + 0.5) / multiple;
}


