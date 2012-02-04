/**
 * shooter.h
 * 
 * Operates device that shoots basketballs by spinning wheels.
 * 
 * Skyline Spartabots, Team 2976
 * Made for 2012 Robot Rumble
 */

#include "shooter.h"

Shooter::Shooter(SpeedController *SpeedController1, SpeedController *SpeedController2, Joystick *Joystick, RangeFinder *RangeFinder)
{
	mSpeedController1 = SpeedController1;
	mSpeedController2 = SpeedController2;
	mJoystick = Joystick;
	mRangeFinder = RangeFinder;
}

/**
 * Shooter::Shoot
 * 
 * Finds position of robot, and gives it to Shooter::CalculateSpeed, 
 * from which it gets the correct speed for the wheels to turn. 
 * When the joystick trigger is pulled, 
 * the wheels spin at the speed from Shooter::CalculateSpeed, 
 * and the ball is loaded into the shooter.
 * 
 * Input:
 * 	-None
 * 
 * Output:
 * 	-None
 * 	
 * Side-effects:
 * 	-None
 */


void Shooter::Shoot() {
	float position = mRangeFinder->FromWallInches();
	float speed = Shooter::CalculateSpeed(position);
	
	bool trigger_state = mJoystick->GetTrigger();
	
	if (trigger_state) {
		mSpeedController1->Set(speed);
		mSpeedController2->Set(-speed); // todo make sure wheels are spinning correctly
		LoadBall();
	}
}

void Shooter::LoadBall() {
	// empty
}

/**
 * Shooter::CalculateSpeed
 * 
 * Receives position of robot from Shooter::Shoot,
 * and performs calculations to find the speed at which the wheels
 * need to turn in order to shoot the ball the correct distance.
 * 
 * Input:
 * 	-None
 * 
 * Output:
 * 	-speed
 * 	
 * Side-effects:
 * 	-None
 */

float Shooter::CalculateSpeed(float position) {
	return 0.0;
}
