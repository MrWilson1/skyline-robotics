/**
 * shooter.h
 * 
 * Operates device that shoots basketballs by spinning wheels.
 * 
 * Skyline Spartabots, Team 2976
 * Made for 2012 Robot Rumble
 */

#include "shooter.h"
#include <cmath>

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
 * 	-position
 * 
 * Output:
 * 	-speed
 * 	
 * Side-effects:
 * 	-None
 */

float Shooter::CalculateSpeed(float position) {
	float height = kBasketHeight - kShooterHeight;
	float distance; // do something with position
		
	// calculates how fast the ball needs to be (initialVelocity) as it leaves the shooter
	float initialVelocityNum = -gravity * distance * distance;
	float initialVelocityDenom = 2 * ( height - ( distance * tan(kAngle) ) * ( cos(kAngle) ) * ( cos (kAngle) ) ); 
	float initialVelocity = sqrt ( initialVelocityNum / initialVelocityDenom );
		
	float speed; // calculate how fast wheels need to spin in order to accelerate ball to initialVelocity
	
	speed = initialVelocity; // Temporary, to get rid of the 'unused variable' warning.  todo: delete or modify this line.
	return 0.0;
}
