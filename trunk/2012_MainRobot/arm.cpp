#include "arm.h"

/**
 * @brief Constructor for Arm class.
 * 
 * @param[in] armMotor A pointer to the motor that controls the arm.
 * @param[in] topLimit A pointer to the top limit switch.
 * @param[in] bottomLimit A pointer to the bottom limit switch.
 */
Arm::Arm (
		SpeedController *armMotor,
		DigitalInput *topLimit,
		DigitalInput *bottomLimit)
{
	mArmMotor = armMotor;
	mTopLimit = topLimit;
	mBottomLimit = bottomLimit;
	
	mMotorWatchdog = new MotorLimitWatchdog(
			"Arm",
			mArmMotor,
			mTopLimit,
			mBottomLimit);
}

/**
 * @brief Deconstructor for the arm class.
 * 
 * @details
 * Used to kill the motor watchdog.
 */
Arm::~Arm ()
{
	if (mMotorWatchdog) {
		mMotorWatchdog->Stop();
		delete mMotorWatchdog;
	}
	
	if (mArmMotor) {
		delete mArmMotor;
	}
	
	if (mTopLimit) {
		delete mTopLimit;
	}
	
	if (mBottomLimit) {
		delete mBottomLimit;
	}
}

/**
 * @brief Makes the arm go up. 
 * 
 * @details
 * If the top limit switch is pressed, then the arm stops going up.
 * In other words, when the arm is at maximum height, it stops automatically.
 */
void Arm::GoUp()
{
	if (mTopLimit->Get()) {
		mArmMotor->Set(0);
	} else {
		mArmMotor->Set(kMotorSpeed * kMotorDirection);
	}
}

/**
 * @brief Makes the arm go down.
 * 
 * @details
 * If the bottom limit switch is pressed, then the arms stop going down.
 * In other words, when the arm is at minimum height, it stops automatically.
 */
void Arm::GoDown()
{
	if (mBottomLimit->Get()) {
		mArmMotor->Set(0);
	} else {
		mArmMotor->Set(kMotorSpeed * kMotorDirection * -1);
	}
}

/**
 * @brief Makes the arm stop.
 */
void Arm::Stop() {
	mArmMotor->Set(0);
}


/**
 * @brief Creates an instance of this class.
 * 
 * @param[in] arm Pointer to the arm.
 * @param[in] joystick Pointer to the joystick.
 */
ArmController::ArmController (Arm *arm, Joystick *joystick) {
	mArm = arm;
	mJoystick = joystick;
}

/**
 * @brief Provides a thin layer to control the arm using a 
 * joystick.  
 * 
 * @details
 * While button 4 is pushed, the arm goes up. 
 * While button 5 is pushed, the arm goes down.
 * While neither button is pushed, the arm stops.
 */
void ArmController::Run() {
	bool armUp = mJoystick->GetRawButton(4);
	bool armDown = mJoystick->GetRawButton(5);
	if ( armUp ) {
		mArm->GoUp();
	} else if ( armDown ) {
		mArm->GoDown();
	} else {
		mArm->Stop();
	}
}
