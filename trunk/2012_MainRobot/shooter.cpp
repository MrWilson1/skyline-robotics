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

Shooter::Shooter(
		SpeedController *topLeftSpeedController, 
		SpeedController *topRightSpeedController,
		SpeedController *bottomLeftSpeedController,
		SpeedController *bottomRightSpeedController, 
		RangeFinder *rangeFinder)
{
	mTopLeftSpeedController = topLeftSpeedController;
	mTopRightSpeedController = topRightSpeedController;
	mBottomLeftSpeedController = bottomLeftSpeedController;
	mBottomRightSpeedController = bottomRightSpeedController;
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
void Shooter::SetSpeedManually(float speed)
{
	float slowSpeed = speed * kReductionFactor;
	
	mTopLeftSpeedController->Set(-1 * slowSpeed);
	mTopRightSpeedController->Set(slowSpeed);
	mBottomLeftSpeedController->Set(-1 * speed);
	mBottomRightSpeedController->Set(speed);
}

void Shooter::SetSpeedAutomatically()
{
	float distance = Shooter::CalculateDistance();
	float speed = Shooter::CalculateSpeed(distance);
	
	SmartDashboard::GetInstance()->Log(speed, "Shooter speed: ");
	
	float coercedSpeed = Tools::Coerce(speed, kMinSpeed, kMaxSpeed, 0, 1);
	
	SmartDashboard::GetInstance()->Log(coercedSpeed, "Coerced speed: ");
	
	Shooter::SetSpeedManually(coercedSpeed);
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
	float pi = 4 * atan(1);
	float height = kBasketHeight - kShooterHeight;
	float angle = ( kShooterAngle * 2 * pi ) / ( 360 ); // converts from degrees to radians
		
	// calculates how fast the ball needs to be (initialVelocity) as it leaves the shooter
	float gravity = kGravity;
	float initialVelocityNum = gravity * distance * distance;
	float initialVelocityDenom = 2 * ( height - ( distance * tan(angle) ) ) * ( cos(angle) ) * ( cos(angle) ); 
	float initialVelocity = sqrt ( initialVelocityNum / initialVelocityDenom );
		
	float speed = initialVelocity / 336;
	// todo: experiment to find actual maximum initial velocity to which wheels can accelerate ball
	return speed;
}

ShooterController::ShooterController(Shooter *shooter, Joystick *joystick)
{
	mShooter = shooter;
	mJoystick = joystick;
}

/**
 * ShooterController::Run
 * 
 * Uses manual and preset modes for the shooter depending on joystick input.
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

void ShooterController::Run(void)
{
	bool setToManual = mJoystick->GetRawButton(2); // manual	
	bool setToPreset = mJoystick->GetTrigger(); // preset
	float throttle = mJoystick->GetThrottle();
	
	SmartDashboard::GetInstance()->Log(setToManual, "Set to manual? ");
	SmartDashboard::GetInstance()->Log(setToPreset, "Set to preset? ");
	SmartDashboard::GetInstance()->Log(throttle, "Shooter Throttle: ");
	
	if ( setToManual )
	{
		mShooter->SetSpeedManually(throttle);
	} else if ( setToPreset ) {
		mShooter->SetSpeedAutomatically();
	} else {
		mShooter->SetSpeedManually(0);
	}
}
