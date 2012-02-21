#include "elevator.h"

/**
 * @brief Creates an instance of this class.
 * 
 * @param[in] middleLimitSwitch Pointer to the middle limit switch.
 * @param[in] topLimitSwitch Pointer to the top limit switch.
 * @param[in] speedController Pointer to the elevator speed controller.
 */
Elevator::Elevator(DigitalInput *middleLimitSwitch, DigitalInput *topLimitSwitch, SpeedController *speedController)
{
	mMiddleLimitSwitch = middleLimitSwitch;
	mTopLimitSwitch = topLimitSwitch;
	mSpeedController = speedController;
}

/*
 * @brief Checks to see if there is a ball
 * in the middle of the elevator.
 */
bool Elevator::IsBallIn(void) {
	return (bool) mMiddleLimitSwitch->Get();
}

/**
 * @brief Checks to see if there is a ball
 * at the top of the elevator.
 */
bool Elevator::IsBallAtTop(void) {
	return (bool) mTopLimitSwitch->Get();
}

/**
 * @brief Makes the elevator stop.
 * 
 * @details
 * In other words, sets the elevator motor speed to 0.0.
 */
void Elevator::Stop(void) {
	mSpeedController->Set(0.0);
}

/**
 * @brief Makes the elevator move.
 * 
 * @details
 * In other words, sets the elevator motor speed to default speed.
 */
void Elevator::MoveUp(void) {
	mSpeedController->Set(kDefaultSpeed);
}

/**
 * @brief Makes an instance of this class.
 * 
 * @param[in] elevator Pointer to Elevator object.
 * @param[in] joystick Pointer to joystick.
 */
ElevatorController::ElevatorController(Elevator *elevator, Joystick *joystick)
{
	mElevator = elevator;
	mJoystick = joystick;
}

/**
 * @brief Provides a thin layer to control the elevator
 * using a joystick.
 * 
 * @details
 * Button 8 causes the conveyor belt to move the ball up
 * until it reaches the top. The driver must press button 9
 * to continue moving the conveyor belt, i.e. load the ball
 * into the shooter. 
 * 
 * A limit switch at the middle of the conveyor belt 
 * tells the driver if there is a ball at the middle 
 * of the conveyor belt; this way, the driver has room 
 * to decide to pick up another ball or continue
 * moving the belt (and shoot the ball).
 */
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
