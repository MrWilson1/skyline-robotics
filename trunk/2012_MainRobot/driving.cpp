#include "driving.h"


/**
 * @brief An empty constructor -- currently does nothing.
 */
BaseJoystickController::BaseJoystickController():
		BaseController()
{
	// Empty
}

/**
 * @brief Squares the input of any number to allow more
 * sensitive driving when the joystick is pushed less.
 * 
 * @param[in] number The number to square.  Should be
 * between -1.0 and 1.0
 * 
 * @returns The squared number (between -1.0 and 1.0, 
 * depending on the original number)
 */
float BaseJoystickController::squareInput(float number)
{
	return (number < 0) ? - (number * number) : (number *number);
}

/**
 * @brief Gets the magnitude of the robot based on the
 * throttle.
 * 
 * @param[in] joystick A pointer to the joystick to get the 
 * throttle value from.
 * 
 * @returns A number between BaseJoystickController::kSpeedFactorMin 
 * BaseJoystickController::kSpeedFactorMax. 
 */
float BaseJoystickController::getSpeedFactor(Joystick *joystick)
{
	float rawFactor = joystick->GetThrottle();
	float normalizedFactor = Tools::Coerce(
			rawFactor,
			-1.0,
			1.0,
			kSpeedFactorMin,
			kSpeedFactorMax);
	return normalizedFactor;
}



TestMotor::TestMotor(Joystick *joystick, SpeedController *speedController)
{
	mJoystick = joystick;
	mSpeedController = speedController;
}

void TestMotor::Run()
{
	float speed = mJoystick->GetY();
	SmartDashboard::GetInstance()->Log(speed, "TestMotor::speed");
	mSpeedController->Set(speed);
}


MinimalistDrive::MinimalistDrive(RobotDrive *robotDrive) :
		BaseController()
{
	mRobotDrive = robotDrive;
}

void MinimalistDrive::Run()
{
	mRobotDrive->TankDrive(0.0, 0.0);
}


/**
 * @brief A method of driving the robot via TankDrive with two
 * joysticks
 * 
 * @param[in] robotDrive A pointer to the RobotDrive object created 
 * in MyRobot::InitiateHardware
 * 
 * @param[in] leftJoystick A pointer to the left joystick 
 * (controls the left side of the robot)
 * 
 * @param[in] rightJoystick A pointer to the right joystick 
 * (controls the right side of the robot)
 */
TankJoysticks::TankJoysticks(RobotDrive *robotDrive, Joystick *leftJoystick, Joystick *rightJoystick):
		BaseJoystickController()
{
	mRobotDrive = robotDrive;
	mLeftJoystick = leftJoystick;
	mRightJoystick = rightJoystick;
}

/**
 * @brief This method is called automatically during MyRobot::OperatorControl
 * 
 * @details
 * It should contain no 'Wait' statements or excessive loops.
 * 
 * This will currently also log the left and right speed values to the
 * SmartDashboard. 
 */
void TankJoysticks::Run(void)
{
	SmartDashboard::GetInstance()->Log("Alive", "Tank status");
	float left = mLeftJoystick->GetY();
	float right = mRightJoystick->GetY(); 
	
	float squaredLeft = squareInput(left);
	float squaredRight = squareInput(right);
	
	float speedFactor = getSpeedFactor(mLeftJoystick);
	
	squaredLeft *= speedFactor;
	squaredRight *= speedFactor;
	
	SmartDashboard::GetInstance()->Log(squaredLeft, "TankJoysticks::squaredLeft");
	SmartDashboard::GetInstance()->Log(squaredRight, "TankJoysticks::squaredRight");
	SmartDashboard::GetInstance()->Log(speedFactor, "TankJoysticks::speedFactor");
	
	mRobotDrive->TankDrive(squaredLeft, squaredRight);
}



/**
 * @brief A method of controlling the robot by using only
 * one joystick.
 * 
 * @details
 * This is a form of controlling the robot by using the single
 * Extreme 3D Pro joystick.  This differs from our other
 * joysticks because it comes with twisting motions.
 * 
 * @param[in] robotDrive A pointer to the robotdrive
 * @param[in] joystick A pointer to the joystick (use the Extreme
 * 3d Pro joystick)
 */
SingleJoystick::SingleJoystick(RobotDrive *robotDrive, Joystick *joystick)
{
	mRobotDrive = robotDrive;
	mJoystick = joystick;
}

/**
 * @brief This method is called automatically during MyRobot::OperatorControl
 * 
 * @details
 * It should contain no 'Wait' statements or excessive loops.
 * 
 * This will currently also log values to the SmartDashboard.
 */
void SingleJoystick::Run()
{
	float x = mJoystick->GetX();
	float y = mJoystick->GetY();
	
	float squaredX = squareInput(x);
	float squaredY = squareInput(y);
	
	// Temporary
	float speedFactor = GetSpeedDecreaseFactor();	
	
	squaredY *= speedFactor;
	squaredX *= speedFactor;
	
	SmartDashboard::GetInstance()->Log(squaredX, "SingleJoystick::squaredX");
	SmartDashboard::GetInstance()->Log(squaredY, "SingleJoystick::squaredY");
	SmartDashboard::GetInstance()->Log(speedFactor, "SingleJoystick::speedFactor");
		
	mRobotDrive->ArcadeDrive(squaredY,-squaredX);
}

/**
 * @brief Temporary.
 */

float SingleJoystick::GetSpeedDecreaseFactor(void)
{
	float rawFactor = mJoystick->GetTwist();
	float normalizedFactor = Tools::Coerce(
			rawFactor,
			-1.0,
			1.0,
			kSpeedFactorMin,
			kSpeedFactorMax);
	return normalizedFactor;
}



/**
 * @brief A way to control the robot during Hybrid mode,
 * using the angle of the hands to the shoulders on the 
 * Y-axis.
 * 
 * @param[in] robotDrive A pointer to the robotdrive.
 * @param[in] kinect A pointer to the kinect.
 */
BaseKinectController::BaseKinectController(
		RobotDrive *robotDrive,
		Kinect *kinect) :
		BaseController()
{
	mRobotDrive = robotDrive;
	mKinect = kinect;
}

/**
 * @brief Checks to see if the player is ready to
 * operate the robot.
 * 
 * @todo
 * This assumes that the x-value of the left hand is lesser
 * then the x-value of the right hand.  This assumption 
 * needs to be tested.
 * 
 * @returns Returns false if:
 *   - The skeleton is not being tracked
 *   - The player's hands are crossed (relative 
 *     to each other)
 *     
 *
 */
bool BaseKinectController::IsPlayerReady()
{	
	bool isPlayerTracked = mKinect->GetTrackingState() == Kinect::kTracked;
	
	float leftX = mKinect->GetSkeleton().GetWristLeft().x;
	float rightX = mKinect->GetSkeleton().GetWristRight().y;
	bool areHandsCrossed = leftX > rightX;
	
	return isPlayerTracked or areHandsCrossed;
}

/**
 * @brief Halts the robot.
 */
void BaseKinectController::HaltRobot(void)
{
	mRobotDrive->TankDrive(0.0, 0.0);
}

/**
 * @brief A way to control the robot by using the Kinect during
 * Hybrid mode, using the z-distance of the hands to control 
 * the robot.
 * 
 * @param[in] robotDrive A pointer to the robotdrive.
 * @param[in] kinect A pointer to the Kinect.
 */
KinectController::KinectController(RobotDrive *robotDrive, Kinect *kinect):
		BaseKinectController(robotDrive, kinect)
{
	// Empty.
}

/**
 * @brief This method is called automatically during MyRobot::OperatorControl
 * 
 * @details
 * It should contain no 'Wait' statements or excessive loops.
 * 
 * The left hand controls the left treads.  The right hand controls the
 * right treads.  The closer the hands are to the robot, the faster
 * that side spins.  Pulling your hands back will soon make the robot back up.
 * 
 * The moment any of your hands are between the shoulders, the robot will
 * freeze.  Therefore, the safest way to immediately halt the robot is 
 * to suddenly cross your arms in an 'X' shape.
 * 
 * Raising both your hands up will make the robot shoot (like shooting a
 * basketball)
 * 
 * This will currently also log values to the SmartDashboard.
 */
void KinectController::Run(void)
{
	bool isPlayerReady = IsPlayerReady();
	SmartDashboard::GetInstance()->Log(isPlayerReady, "KinectController::IsReady()");
	
	if (isPlayerReady) {
		float left = GetLeftArmDistance();
		float right = GetRightArmDistance();
		mRobotDrive->TankDrive(left, right);
			
		SmartDashboard::GetInstance()->Log(left, "KinectController::GetLeftArmDistance()");
		SmartDashboard::GetInstance()->Log(right, "KinectController::GetRightArmDistance()");
	} else {
		HaltRobot();		
	}
	
	bool isShooting = IsPlayerShooting();
	SmartDashboard::GetInstance()->Log(isShooting, "KinectController::IsPlayerShooting()");
	if (isShooting) {
		// Empty
	} else {
		// Empty
	}
}

/**
 * @brief Returns how far the left hand is.
 * 
 * @returns A number between -1.0 and 1.0.
 * The closer the hand is to the Kinect, the higher the 
 * returned value.
 */
float KinectController::GetLeftArmDistance(void)
{
	float originJoint = mKinect->GetSkeleton().GetShoulderLeft().z;
	float movingJoint = mKinect->GetSkeleton().GetWristLeft().z;
	
	float distance = originJoint - movingJoint;
	float output = Tools::Coerce(distance, kArmMinZ, kArmMaxZ, -1, 1);
	
	SmartDashboard::GetInstance()->Log(output, "Left movement");
	
	return output;
}

/**
 * @brief Returns how far the right hand is.
 * 
 * @returns A number between -1.0 and 1.0.
 * The closer the hand is to the Kinect, the higher the 
 * returned value.
 */
float KinectController::GetRightArmDistance(void)
{
	float originJoint = mKinect->GetSkeleton().GetShoulderRight().z;
	float movingJoint = mKinect->GetSkeleton().GetWristRight().z;
	
	float distance = originJoint - movingJoint;
	float output = Tools::Coerce(distance, kArmMinZ, kArmMaxZ, -1, 1);
	
	SmartDashboard::GetInstance()->Log(output, "Right movement");
	
	return output;	
}

/**
 * @brief Checks to see if the player is shooting.
 * 
 * @details
 * The robot will check to see if the player has made
 * a 'basketball shooting' movement to shoot
 * the ball.
 * 
 * @returns 'true' if the player's hands are
 * elevated above the head; 'false' if they aren't.
 */
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
 * @brief Constructor for KinectAngleController class.
 * 
 * @param[in] robotDrive Pointer to RobotDrive object.
 * @param[in] leftKinectStick Pointer to left Kinectstick.
 * @param[in] rightKinectStick Pointer to right Kinectstick.
 * @param[in] kinect Pointer to the Kinect.
 */
KinectAngleController::KinectAngleController(RobotDrive *robotDrive, KinectStick *leftKinectStick, 
		KinectStick *rightKinectStick, Kinect *kinect) :
		BaseKinectController(robotDrive, kinect)
{
	mLeftKinectStick = leftKinectStick;
	mRightKinectStick = rightKinectStick;
}

/**
 * @brief Checks to see if player has made a pushing-forward motion to shoot.
 * 
 * @returns True if both of the player's wrists are 
 * a certain distance (0.3) away from his or her shoulders.
 * 
 * False otherwise.
 */
bool KinectAngleController::IsManuallyShooting(void)
{
	float rightOrigin = mKinect->GetSkeleton().GetShoulderRight().z;
	float leftOrigin = mKinect->GetSkeleton().GetShoulderLeft().z;
	float rightMoving = mKinect->GetSkeleton().GetWristRight().z;
	float leftMoving = mKinect->GetSkeleton().GetWristLeft().z;
	
	float rightDelta = rightMoving - rightOrigin;
	float leftDelta = leftMoving - leftOrigin;
	
	// Pushing forward to shoot.
	if ((fabs(rightDelta) > kPushThreshold) and (fabs(leftDelta) > kPushThreshold)) {
		return true;
	} else {
		return false;
	}
}

/**
 * @brief This method is called automatically during MyRobot:AutonomousControl.
 * 
 * @details
 * Uses KinectStick class to drive the robot like a tank.
 * 
 * When the arms are straight out (perpendicular to the rest of the body), 
 * the speed of the robot is set to 0.0.
 * Raising the arms in an arc increases the speed forward.
 * Lowering the arms in an arc increases the speed backward.
 * 
 * The robot will freeze when either of the driver's hands is between
 * the shoulders. Therefore, the easiest way to stop the robot
 * is to cross one's arms in an 'X' shape.
 * 
 * Moving both one's wrists a certain distance forward (0.3) 
 * will tell the robot to shoot.
 * 
 * This will log values to SmartDashboard.
 */
void KinectAngleController::Run(void)
{
	bool isPlayerReady = IsPlayerReady();
	SmartDashboard::GetInstance()->Log(isPlayerReady, "KinectAngleController::IsReady()");
	
	if ( isPlayerReady ) {
		float rightY = mRightKinectStick->GetY();
		float leftY = mLeftKinectStick->GetY();
		SmartDashboard::GetInstance()->Log(rightY, "Right Kinectstick y-value: ");
		SmartDashboard::GetInstance()->Log(leftY, "Left Kinectstick y-value: ");
		mRobotDrive->TankDrive(leftY * kSpeedDecreaseFactor * -1, rightY * kSpeedDecreaseFactor * -1);
	} else {
		HaltRobot();
	}
	
	bool isShooting = IsManuallyShooting();
	SmartDashboard::GetInstance()->Log(isShooting, "KinectAngleController::IsManuallyShooting()");
	if (isShooting) {
		// empty
	} else {
		// empty
	}
}
