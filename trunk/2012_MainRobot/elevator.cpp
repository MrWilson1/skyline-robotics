#include "elevator.h"

/**
 * @brief Creates an instance of this class.
 */
Elevator::Elevator(
		SpeedController *speedController, 
		BallTransfer *ballTransfer) :
		BaseComponent()
{
	mSpeedController = speedController;
	mBallTransfer = ballTransfer;
}

/**
 * @brief Makes the elevator move up.
 * 
 * @details
 * This needs to be called repeatedly, perhaps inside Elevator::Run.
 * 
 * Will not work if a ball is at the top, in the basket, or 
 * transitioning between the top and the basket.
 */
bool Elevator::MoveUp(double speed)
{
	if (!mBallTransfer->IsBallAtTop() and !mBallTransfer->IsBallTransfering()) {
		mSpeedController->Set(speed);
		return true;
	} else {
		return false;
	}
}

bool Elevator::MoveUp(void)
{
	return Elevator::MoveUp(kDefaultSpeed);
}

/**
 * Begins moving the Elevator down.  Will not work
 * if the ball is transitioning.
 */
bool Elevator::MoveDown(double speed)
{
	if (!mBallTransfer->IsBallAtTop() and !mBallTransfer->IsBallTransfering()) {
		mSpeedController->Set(-speed);
		return true;
	} else {
		return false;
	}
}

bool Elevator::MoveDown(void)
{
	double defaultSpeed = 1.0;
	return Elevator::MoveDown(defaultSpeed);
}


/**
 * @brief Responsible for transferring the ball
 * from the elevator to the shooter.
 */
BallTransfer::BallTransfer(
		Servo *servo, 
		DigitalInput *topLimitSwitch,
		DigitalInput *shooterLimitSwitch):
	BaseComponent()
{
	mServo = servo;
	mTopLimitSwitch = topLimitSwitch;
	mShooterLimitSwitch = shooterLimitSwitch;
}

/**
 * @brief Tests if the shooter is occupied using
 * the limit swtich
 * 
 * @returns Returns 'true' if a ball is triggering 
 * the limit switch inside the shooter.
 */
bool BallTransfer::IsShooterOccupied()
{
	return (bool) mShooterLimitSwitch->Get();
}

/**
 * @brief Tests if the ball is at the top of
 * the elevator.
 * 
 * @returns Returns 'true' if a ball is triggering
 * the limit switch at the top of the elevator.
 */
bool BallTransfer::IsBallAtTop()
{
	return (bool) mTopLimitSwitch->Get();
}

/**
 * @brief Tests if the ball is in the 
 * process of transferring.
 * 
 * @returns Returns 'true' if the servo is 
 * not at the safe position.
 */
bool BallTransfer::IsBallTransfering()
{
	if (mServo->Get() == kServoSafePosition) {
		return true;
	} else {
		return false;
	}
}

/**
 * @brief Tests to see if every single piece of the
 * ball transfer is ready for a new ball.
 * 
 * @returns Tests if the servo is ready and the top
 * limit switch is not triggering.
 */
bool BallTransfer::IsReady()
{
	return !(IsBallTransfering() or IsBallAtTop() or IsShooterOccupied());
}

/**
 * @brief Sets the angle of the servo to push the ball into the shooter.
 */
bool BallTransfer::StartTransfer()
{
	if (IsReady()) {
		mServo->Set(kServoExtendedPosition);
	}
	return IsShooterOccupied();
}

/**
 * @brief Moves the servo back down.
 */
bool BallTransfer::EndTransfer()
{
	if (!IsBallAtTop()) {
		mServo->Set(kServoSafePosition);
	}
	return IsReady();
}

/**
 * @brief Freezes the transferring.
 */
void BallTransfer::HaltTransfer()
{
	float speed = mServo->Get();
	mServo->Set(speed);
}
