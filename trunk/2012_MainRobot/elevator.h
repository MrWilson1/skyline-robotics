/**
 * elevator.h
 * 
 * Provides code to control and manipulate the elevator, and 
 * the shooter-elevator interface.
 * 
 * Skyline Spartabots, Team 2976
 * Made for 2012 Robot Rumble
 */

#ifndef ELEVATOR_H_
#define ELEVATOR_H_

// 3rd-party libraries
#include "WPILib.h"

class Elevator
{
protected:
	SpeedController *mSpeedController; 	// Controls the belt
	Servo *mBasketServo;				// Can push balls into the basket
	DigitalInput *mTopLimitSwitch;		// Is the ball hitting the top of the lift?
	DigitalInput *mBasketLimitSwitch;	// Is the ball in the basket?
	bool mIsTransitioning;
	bool mIsRecovering;
	
public:
	Elevator(SpeedController *, Servo *, DigitalInput *, DigitalInput *);
	bool MoveUp(void);
	bool MoveUp(double);
	bool MoveDown(void);
	bool MoveDown(double);
	bool IsBallInBasket(void);
	bool IsBallAtTop(void);
	bool PushBallToBasket(void);
	bool IsBallTransitioning(void);
};


#endif
