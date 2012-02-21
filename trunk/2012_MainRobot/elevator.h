/**
 * @file elevator.h
 * 
 * @brief Provides code to control and manipulate the elevator, and 
 * the shooter-elevator interface.
 */

#ifndef ELEVATOR_H_
#define ELEVATOR_H_

// 3rd-party libraries
#include "WPILib.h"

// Our code
#include "components.h"

/**
 * @brief Transfers the ball from the floor to the top of the elevator.
 */
class Elevator : public BaseComponent
{
protected:
	DigitalInput *mMiddleLimitSwitch;
	DigitalInput *mTopLimitSwitch;
	SpeedController *mSpeedController; 	// Controls the belt
	static const float kDefaultSpeed = 1.0;		// -1.0 to 1.0
	
public:
	Elevator(DigitalInput *, DigitalInput *, SpeedController *);
	bool IsBallIn();
	bool IsBallAtTop();
	bool IsShooterOccupied();
	void Stop();
	void MoveUp();
};

class ElevatorController : public BaseController
{
protected:
	Elevator *mElevator;
	Joystick *mJoystick;
	
public:
	ElevatorController (Elevator *, Joystick* );
	void Run();
};


#endif
