/**
 * elevator.h
 * 
 * Provides code to control and manipulate the elevator, and 
 * the shooter-elevator interface.
 * 
 * Skyline Spartabots, Team 2976
 * Made for 2012 Robot Rumble
 */

#include "elevator.h"


Elevator::Elevator(
		SpeedController *speedController, 
		Servo *basketServo, 
		DigitalInput *topLimitSwitch, 
		DigitalInput *basketLimitSwitch)
{
	mSpeedController = speedController;
	mBasketServo = basketServo;
	mTopLimitSwitch = topLimitSwitch;
	mBasketLimitSwitch = basketLimitSwitch;
	
	mIsTransitioning = false;
	mIsRecovering = false;
}

/**
 * Begins moving the elevator up.  This needs
 * to be called repeatedly, perhaps inside Elevator::Run.
 * 
 * Will not work if a ball is at the top, in the basket, or 
 * transitioning between the top and the basket.
 */
bool Elevator::MoveUp(double speed)
{
	if (!IsBallAtTop() and !IsBallTransitioning()) {
		mSpeedController->Set(speed);
		return true;
	} else {
		return false;
	}
	
}

bool Elevator::MoveUp(void)
{
	double defaultSpeed = 1.0;
	return Elevator::MoveUp(defaultSpeed);
}

/**
 * Begins moving the Elevator down.  Will not work
 * if the ball is transitioning.
 */
bool Elevator::MoveDown(double speed)
{
	if (!IsBallTransitioning()) {
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


bool Elevator::IsBallInBasket(void)
{
	return (bool) mBasketLimitSwitch->Get();
}

bool Elevator::IsBallAtTop(void)
{
	return (bool) mTopLimitSwitch->Get();
}

bool Elevator::IsBallTransitioning(void)
{
	return mIsTransitioning and mIsRecovering;
}

/**
 * Todo: write this.
 */
bool Elevator::PushBallToBasket(void)
{
	
	return false;
}
