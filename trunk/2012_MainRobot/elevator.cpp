#include "elevator.h"

Elevator::Elevator(DigitalInput *middleLimitSwitch, DigitalInput *topLimitSwitch, SpeedController *speedController)
{
	mMiddleLimitSwitch = middleLimitSwitch;
	mTopLimitSwitch = topLimitSwitch;
	mSpeedController = speedController;
}

bool Elevator::IsBallIn(void) {
	return (bool) mMiddleLimitSwitch->Get();
}

bool Elevator::IsBallAtTop(void) {
	return (bool) mTopLimitSwitch->Get();
}

void Elevator::Stop(void) {
	mSpeedController->Set(0.0);
}

void Elevator::MoveUp(void) {
	mSpeedController->Set(kDefaultSpeed);
}

ElevatorController::ElevatorController(Elevator *elevator, Joystick *joystick)
{
	mElevator = elevator;
	mJoystick = joystick;
}

void ElevatorController::Run(void) {
	bool moveBall = mJoystick->GetRawButton(8);
	bool loadBall = mJoystick->GetRawButton(9);
	
    if ( moveBall && !mElevator->IsBallAtTop() ) {
    	mElevator->MoveUp();
    } else if ( mElevator->IsBallAtTop() && !loadBall ) {
    	mElevator->Stop();
    } else if ( mElevator->IsBallAtTop() && loadBall ) {
    	mElevator->MoveUp();
    }
    
    SmartDashboard::GetInstance()->Log(mElevator->IsBallIn(), "Ball in middle: ");
    SmartDashboard::GetInstance()->Log(mElevator->IsBallAtTop(), "Ball at top: ");
}
