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
	float xFromWall = mRangeFinder->FromWallInches();
	float yFromWall = mRangeFinder->FromWallInches(); // todo need to add another Ultrasound sensor.
	float angle = mGyro->GetAngle();
	
	float xRawPosition;
	float yRawPosition; // "raw" meaning the angle has not yet been taken into account
	
	if ( cos(angle) < 0 ) {
		xRawPosition = kArenaXMaximum - xFromWall;
		yRawPosition = kArenaYMaximum - yFromWall;
	}
	else {
		xRawPosition = xFromWall;
		yRawPosition = yFromWall;
	}
	
	float xPosition = xRawPosition * fabs(cos(angle));
	float yPosition = yRawPosition * fabs(cos(angle));

	float distance = sqrt( (kBasketYCoordinate - yPosition)*(kBasketYCoordinate - yPosition) + (kBasketXCoordinate - xPosition)*(kBasketXCoordinate - xPosition) );	
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
	/*
	float height = kBasketHeight - kShooterHeight;
	float distance = 100; // do something with position
		
	// calculates how fast the ball needs to be (initialVelocity) as it leaves the shooter
	float initialVelocityNum = -kGravity * distance * distance;
	float initialVelocityDenom = 2 * ( height - ( distance * tan(kAngle) ) * ( cos(kAngle) ) * ( cos (kAngle) ) ); 
	float initialVelocity = sqrt ( initialVelocityNum / initialVelocityDenom );
		
	float speed; // calculate how fast wheels need to spin in order to accelerate ball to initialVelocity
	
	speed = initialVelocity; // Temporary, to get rid of the 'unused variable' warning.  todo: delete or modify this line.
	*/
	return 0.0;
}

void Shooter::Run() {
	// empty
}
