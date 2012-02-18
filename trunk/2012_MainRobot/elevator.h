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
 * @brief Transfers the ball from the elevator to the shooter.
 */
class BallTransfer : public BaseComponent
{
protected:
	Servo *mServo;
	DigitalInput *mTopLimitSwitch;
	DigitalInput *mShooterLimitSwitch;
	static const float kServoSafePosition = 0.0;		// 0.0 to 1.0
	static const float kServoExtendedPosition = 1.0;	// 0.0 to 1.0
	
public:
	BallTransfer(Servo *, DigitalInput *, DigitalInput *);
	bool IsShooterOccupied();
	bool IsBallAtTop();
	bool IsBallTransfering();
	bool IsReady();
	
	bool StartTransfer();
	bool EndTransfer();
	void HaltTransfer();
};


class Elevator : public BaseComponent
{
protected:
	SpeedController *mSpeedController; 	// Controls the belt
	BallTransfer *mBallTransfer;
	static const float kDefaultSpeed = 1.0;		// -1.0 to 1.0
	
public:
	Elevator(SpeedController *, BallTransfer *);
	bool MoveUp();
	bool MoveUp(double);
	bool MoveDown();
	bool MoveDown(double);
};



#endif
