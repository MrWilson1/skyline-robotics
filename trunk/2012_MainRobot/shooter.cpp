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

Shooter::Shooter(SpeedController *speedController1, SpeedController *speedController2, Joystick *joystick, RangeFinder *rangeFinder)
{
	mSpeedController1 = speedController1;
	mSpeedController2 = speedController2;
	mJoystick = joystick;
	mRangeFinder = rangeFinder;
}

/**
 * Shooter::CalculateDistance
 * 
 * Takes position of the robot on the x-axis, position of the robot on the y-axis, and orientation (angle) of the robot to calculate its distance from the hoop.
 * This assumes that one ultrasound sensor is on the north side of the robot, and the other ultrasound sensor is on the west side.
 * 
 * Input:
 * 	-None
 * 
 * Output:
 * 	-distance
 * 	
 * Side-effects:
 * 	-None
 */

float Shooter::CalculateDistance()
{
	float distance = mRangeFinder->FromWallInches();
	return distance;
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
	float distance = Shooter::CalculateDistance();
	float speed = Shooter::CalculateSpeed(distance);
	
	bool setToManual = mJoystick->GetTrigger(); // manual	
	bool setToPreset = mJoystick->GetRawButton(3); // preset
	bool manualFire = mJoystick->GetRawButton(2); // button to fire when shooter is set to manual
	float throttle = mJoystick->GetThrottle();
	
	if (setToPreset) {
		mSpeedController1->Set(speed);
		mSpeedController2->Set(-speed); // todo make sure wheels are spinning correctly
		LoadBall();
	}
	else if (setToManual) {
		mSpeedController1->Set(throttle);
		mSpeedController2->Set(-throttle);
			if ( manualFire ) {
				LoadBall();
			}		
		}
}

/**
 * Shooter::LoadBall
 * 
 * Loads ball into shooter after wheels start turning.
 * Not quite sure how this will work yet.
 * 
 * Input:
 * 	-none
 * 
 * Output:
 * 	-none
 * 	
 * Side-effects:
 * 	-none
 */

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

float Shooter::CalculateSpeed(float distance) {
	float height = kBasketHeight - kShooterHeight;
		
	// calculates how fast the ball needs to be (initialVelocity) as it leaves the shooter
	float initialVelocityNum = -gravity * distance * distance;
	float initialVelocityDenom = 2 * ( height - ( distance * tan(kShooterAngle) ) * ( cos(kShooterAngle) ) * ( cos (kShooterAngle) ) ); 
	float initialVelocity = sqrt ( initialVelocityNum / initialVelocityDenom );
		
	float speed; // calculates how fast wheels need to spin in order to accelerate ball to initialVelocity.
	// todo: experiment to find actual maximum initial velocity to which wheels can accelerate ball
	return speed;
}

void Shooter::Run() {
	Shoot();
}
