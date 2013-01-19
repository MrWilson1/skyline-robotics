#ifndef DRIVE_SUBSYSTEM_H
#define DRIVE_SUBSYSTEM_H
/**
 * Implements various subsystems related to the drive.
 */

#include <string>

#include "WPILib.h"

#include "BaseSubsystem.h"

/**
 * A base drive. All drives must be a subclass of this class.
 * 
 * Note that Holonomic and Mechanum wheel drives are currently
 * not implemented by this interface. 
 * 
 * For the definitions of what these virtual methods are meant to
 * do, please see the WPILib API documentation on the 
 * RobotDrive class. The methods are identical.
 */
class BaseDrive : public BaseSubsystem {
public:
	BaseDrive(const char *name);
	virtual ~BaseDrive();
	
	virtual void Drive(float outputMagnitude, float curve) = 0;
	virtual void TankDrive(float leftValue, float rightValue) = 0;
	virtual void TankDrive(float leftValue, float rightValue, bool squaredInputs) = 0;
	virtual void ArcadeDrive(float moveValue, float rotateValue) = 0;
	virtual void ArcadeDrive(float moveValue, float rotateValue, bool squaredInputs) = 0;
	virtual void TravelDistance(float distanceInInches) = 0;
	virtual void Rotate(float degrees) = 0;
	virtual void StopMoving() = 0;
	virtual void Brake() = 0;
};

/**
 * The absolute simplest drive possible. Contains
 * no PID loops or other forms of feedback systems.
 */
class SimpleDrive : public BaseDrive {
public:
	SimpleDrive(RobotDrive *robotDrive);
	~SimpleDrive();
	
	void Drive(float outputMagnitude, float curve);
	void TankDrive(float leftValue, float rightValue);
	void TankDrive(float leftValue, float rightValue, bool squaredInputs);
	void ArcadeDrive(float moveValue, float rotateValue);
	void ArcadeDrive(float moveValue, float rotateValue, bool squaredInputs);
	void TravelDistance(float distanceInInches);
	void Rotate(float degrees);
	void StopMoving();
	void Brake();
	
private:
	RobotDrive *m_robotDrive;
};

#endif
