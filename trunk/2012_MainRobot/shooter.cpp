#include "shooter.h"

/**
 * @brief Creates an instance of this class.
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
		RangeFinder *rangeFinder) : 
		BaseComponent()
{
	mTopLeftSpeedController = topLeftSpeedController;
	mTopRightSpeedController = topRightSpeedController;
	mBottomLeftSpeedController = bottomLeftSpeedController;
	mBottomRightSpeedController = bottomRightSpeedController;
	mRangeFinder = rangeFinder;
}

/**
 * @brief Calculates the distance from the wall in inches.
 * 
 * @details
 * Uses both the rangefinder and camera to calculate distance
 * 
 * @todo Add code for the camera.
 * 
 * @todo Investigate if the logic in the method needs to be
 * moved elsewhere.
 */
float Shooter::CalculateDistance()
{
	float distance = mRangeFinder->FromWallInches();
	return distance;
}

/**
 * @brief Makes the wheels spin at a certain speed, typically set during manual mode.
 * 
 * @details
 * The input should be in the range -1.0 to 1.0.
 * 
 * @todo
 * Investigate if the motor requires a positive or 
 * negative speed to spin in the correct direction to 
 * shoot.
 * 
 * @param[in] speed The speed of the motor (from -1.0 to 1.0).
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
 * @brief Gets the position of the robot relative to the hoop and
 * attempts to aim and hit it.
 * 
 * @details
 * Takes distance between robot and hoop and
 * determines the necessary initial velocity for the ball. 
 * Converts the velocity to a valid value for the
 * speed controller and passes that value to 
 * Shooter::SetSpeedManually to set the wheels to that speed.
 * 
 * @todo
 * Investigate if this needs to be moved into another class.
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
 * @brief Calculates the speed the motors need to turn based on 
 * the distance to the wall.
 * 
 * @details
 * Receives position of robot from Shooter::Shoot,
 * and performs calculations to find the speed at which the wheels
 * need to turn in order to shoot the ball the correct distance.
 * 
 * @param[in] distance Distance from shooter to hoop in inches.
 *
 * @returns Returns the speed the speedControllers need to
 * turn to fire the ball and hit the hoop (from -1.0 to 1.0).
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
		
	float speed = initialVelocity / kMaxInitialVelocity;
	// todo: experiment to find actual maximum initial velocity to which wheels can accelerate ball
	
	return speed;
}




/**
 * @brief Constructor for ShooterController class.
 * 
 * @param[in] shooter Pointer to shooter.
 * @param[in] joystick Pointer to joystick.
 */
ShooterController::ShooterController(Shooter *shooter, Joystick *joystick) :
		BaseController()
{
	mShooter = shooter;
	mJoystick = joystick;
}

/**
 * @brief Contains code to control the shooter manually and automatically.
 * 
 * @details
 * Uses a joystick for manual mode -- use button 2 to fire normally
 * and the trigger for the preset.
 */

void ShooterController::Run(void)
{
	bool setToManual = mJoystick->GetRawButton(2); // manual	
	bool setToPreset = mJoystick->GetTrigger(); // preset
	float throttle = mJoystick->GetThrottle();
	
	// float twist = mJoystick->GetTwist();
	// float z = mJoystick->GetZ();
	// SmartDashboard::GetInstance()->Log(setToManual, "Set to manual? ");
	// SmartDashboard::GetInstance()->Log(setToPreset, "Set to preset? ");
	// SmartDashboard::GetInstance()->Log(twist, "Shooter Twist: ");
	// SmartDashboard::GetInstance()->Log(z, "Shooter z: ");
	
	SmartDashboard::GetInstance()->Log(throttle, "Shooter Throttle: ");
	
	if ( setToManual ) {
		mShooter->SetSpeedManually(throttle);
		// mShooter->SetSpeedManually(twist);
	} else if ( setToPreset ) {
		mShooter->SetSpeedAutomatically();
	} else {
		mShooter->SetSpeedManually(0);
	}
}
