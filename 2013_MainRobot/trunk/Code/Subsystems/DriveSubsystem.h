#ifndef DRIVE_SUBSYSTEM_H
#define DRIVE_SUBSYSTEM_H

#include "WPILib.h"
#include <string>

class BaseDrive : public Subsystem
{
public:
	BaseDrive(const char *name);
	virtual ~BaseDrive();
	
	virtual void Drive(float outputMagnitude, float curve) = 0;
	virtual void TankDrive(float leftValue, float rightValue) = 0;
	virtual void TankDrive(float leftValue, float rightValue, bool squaredInputs) = 0;
	virtual void ArcadeDrive(float moveValue, float rotateValue) = 0;
	virtual void ArcadeDrive(float moveValue, float rotateValue, bool squaredInputs) = 0;
	virtual void TravelDistance(float distanceInInches);
	virtual void Rotate(float degrees); 
	
	virtual void StopMoving() = 0;
	virtual void Brake() = 0;
};

class SimpleDrive : public BaseDrive
{
public:
	SimpleDrive(RobotDrive *robotDrive);
	
	virtual void Drive(float outputMagnitude, float curve);
	virtual void TankDrive(float leftValue, float rightValue);
	virtual void TankDrive(float leftValue, float rightValue, bool squaredInputs);
	virtual void ArcadeDrive(float moveValue, float rotateValue);
	virtual void ArcadeDrive(float moveValue, float rotateValue, bool squaredInputs);
	virtual void TravelDistance(float distanceInInches);
	virtual void Rotate(float degrees);
	
	virtual void StopMoving();
	virtual void Brake();
	
private:
	RobotDrive *m_robotDrive;
};

#endif
