#include "DriveSubsystem.h"

BaseDrive::BaseDrive(const char *name) :
	Subsystem(name)
{
	// nothing
}

BaseDrive::~BaseDrive() 
{
	// nothing
}

void BaseDrive::Drive(float outputMagnitude, float curve)
{
	// Empty
}

void BaseDrive::TankDrive(float leftValue, float rightValue)
{
	SmartDashboard::PutString("BaseDrive TankDrive 1", "AIII");
}

void BaseDrive::TankDrive(float leftValue, float rightValue, bool squaredInputs)
{
	SmartDashboard::PutString("BaseDrive TankDrive 2", "AIII");
}

void BaseDrive::ArcadeDrive(float moveValue, float rotateValue)
{
	
}

void BaseDrive::ArcadeDrive(float moveValue, float rotateValue, bool squaredInputs)
{
	
}

void BaseDrive::TravelDistance(float distanceInInches)
{
	
}

void BaseDrive::Rotate(float degrees)
{
	
}

void BaseDrive::StopMoving()
{
	
}

void BaseDrive::Brake()
{
	
}


SimpleDrive::SimpleDrive(RobotDrive *robotDrive) :
		BaseDrive("SimpleDrive")
{
	m_robotDrive = robotDrive;
}

void SimpleDrive::Drive(float outputMagnitude, float curve)
{
	m_robotDrive->Drive(outputMagnitude, curve);
}

void SimpleDrive::TankDrive(float leftValue, float rightValue)
{
	m_robotDrive->TankDrive(leftValue, rightValue);
}

void SimpleDrive::TankDrive(float leftValue, float rightValue, bool squaredInputs)
{
	m_robotDrive->TankDrive(leftValue, rightValue, squaredInputs);
}

void SimpleDrive::ArcadeDrive(float moveValue, float rotateValue)
{
	m_robotDrive->ArcadeDrive(moveValue, rotateValue);
}

void SimpleDrive::ArcadeDrive(float moveValue, float rotateValue, bool squaredInputs)
{
	m_robotDrive->ArcadeDrive(moveValue, rotateValue, squaredInputs);
}

void SimpleDrive::TravelDistance(float distanceInInches)
{
	// do nothing
}

void SimpleDrive::Rotate(float degrees)
{
	// do nothing
}



/**
 * Simply tells the wheels to stop spinning.
 */
void SimpleDrive::StopMoving()
{
	float stopValue = 0.0;
	m_robotDrive->TankDrive(stopValue, stopValue);
}

/**
 * Tells the robot to stop moving, accounting for things like
 * slipping and sliding.
 * 
 * On the SimpleDrive, this is equivalent to calling 
 * SimpleDrive::StopMoving.
 */
void SimpleDrive::Brake()
{
	float stopValue = 0.0;
	m_robotDrive->TankDrive(stopValue, stopValue);
}