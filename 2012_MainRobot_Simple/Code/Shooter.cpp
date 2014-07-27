#include "shooter.h"

Shooter::Shooter(
	Jaguar *topLeftSpeedController, 
	Jaguar *topRightSpeedController,
	Jaguar *bottomLeftSpeedController,
	Jaguar *bottomRightSpeedController)
{
	mTopLeftSpeedController = topLeftSpeedController;
	mTopRightSpeedController = topRightSpeedController;
	mBottomLeftSpeedController = bottomLeftSpeedController;
	mBottomRightSpeedController = bottomRightSpeedController;
}

void Shooter::SetSpeed(float speed)
{
	float slowSpeed = speed * kReductionFactor;
	
	mTopLeftSpeedController->Set(slowSpeed);
	mTopRightSpeedController->Set(-1 * slowSpeed);
	mBottomLeftSpeedController->Set(-1 * speed);
	mBottomRightSpeedController->Set(speed);
}

void Shooter::SetSpeed(float upperSpeed, float lowerSpeed)
{
	mTopLeftSpeedController->Set(upperSpeed);
	mTopRightSpeedController->Set(upperSpeed * -1);
	mBottomLeftSpeedController->Set(lowerSpeed * -1);
	mBottomRightSpeedController->Set(lowerSpeed);
}
