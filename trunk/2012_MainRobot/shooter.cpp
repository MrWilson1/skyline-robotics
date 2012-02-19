#include "shooter.h"
#include <cmath>

/**
 * @brief Constructor for shooter class.
 * 
 * @param[in] topLeftSpeedController Pointer to the top left speed controller.
 * @param[in] topRightSpeedController Pointer to the top right speed controller.
 * @param[in] bottomLeftSpeedController Pointer to the bottom left speed controller.
 * @param[in] bottomRightSpeedController Pointer to the bottom right speed controller.
 * @param[in] rangeFinder Pointer to a RangeFinder instance.
 */

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
 * @brief Calculates distance of robot from the hoop.
 * 
 * @returns Distance of robot from the hoop in inches.
 */

float Shooter::CalculateDistance()
{
	float distance = mRangeFinder->FromWallInches();
	return distance;
}

/**
 * @brief Makes the wheels spin at a certain speed, set manually in Manual mode.
 * 
 * @param[in] speed Manually set speed.
 */

void Shooter::SetSpeedManually(float speed)
{
	float slowSpeed = speed * kReductionFactor;
	
	mTopLeftSpeedController->Set(-1 * slowSpeed);
	mTopRightSpeedController->Set(slowSpeed);
	mBottomLeftSpeedController->Set(-1 * speed);
	mBottomRightSpeedController->Set(speed);
}

/**
 * @brief Makes the wheels spin at a certain speed, 
 * set automatically in Preset mode.
 * 
 * @details
 * Takes distance between robot and hoop and
 * determines the necessary initial velocity for the ball. 
 * Converts the velocity to a valid value for the
 * speed controller and passes that value to 
 * Shooter::SetSpeedManually to set the wheels to that speed.
 * 
 */

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
 * @brief Calculates how fast the ball needs to be as it leaves the shooter
 * depending on the distance between the shooter and the hoop.
 * 
 * @param[in] distance Distance from shooter to hoop.
 * 
 * @returns Necessary initial velocity for ball.
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

/**
 * @brief Constructor for ShooterController class.
 * 
 * @param[in] shooter Pointer to shooter.
 * @param[in] joystick Pointer to joystick.
 */

ShooterController::ShooterController(Shooter *shooter, Joystick *joystick)
{
	mShooter = shooter;
	mJoystick = joystick;
}

/**
 * @brief Uses manual and preset modes for the shooter depending on joystick input.
 */

void ShooterController::Run(void)
{
	bool setToManual = mJoystick->GetRawButton(2); // manual	
	bool setToPreset = mJoystick->GetTrigger(); // preset
	// float throttle = mJoystick->GetThrottle();
	
	float twist = mJoystick->GetTwist();
	float z = mJoystick->GetZ();
	SmartDashboard::GetInstance()->Log(setToManual, "Set to manual? ");
	SmartDashboard::GetInstance()->Log(setToPreset, "Set to preset? ");
	SmartDashboard::GetInstance()->Log(twist, "Shooter Twist: ");
	SmartDashboard::GetInstance()->Log(z, "Shooter z: ");
	
	// SmartDashboard::GetInstance()->Log(throttle, "Shooter Throttle: ");
	
	if ( setToManual ) {
		// mShooter->SetSpeedManually(throttle);
		mShooter->SetSpeedManually(twist);
	} else if ( setToPreset ) {
		mShooter->SetSpeedAutomatically();
	} else {
		mShooter->SetSpeedManually(0);
	}
}
