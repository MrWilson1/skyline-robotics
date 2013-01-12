#ifndef DRIVE_COMMANDS_H
#define DRIVE_COMMANDS_H

#include "WPILib.h"
#include "..\Tools\Units.h"

#include "..\Subsystems\DriveSubsystem.h"

class TankDrive : public Command
{
public:
	TankDrive(float leftValue, float rightValue);
	virtual void Initialize();
	virtual void Execute();
	virtual bool IsFinished();
	virtual void End();
	virtual void Interrupted();
	
private:
	BaseDrive m_drive;
	float m_leftValue;
	float m_rightValue;
};

class ArcadeDrive : public Command
{
public:
	ArcadeDrive(float moveValue, float rotateValue);
	virtual ~TankDrive();
	
	virtual void Initialize();
	virtual void Execute();
	virtual bool IsFinished();
	virtual void End();
	virtual void Interrupted();
	
private:
	BaseDrive m_drive;
	float m_moveValue;
	float m_rotateValue;
};

class TravelDistance : public Command
{
public:
	TravelDistance(BaseDrive *drive, float distance, Units unit);
	TravelDistance(BaseDrive *drive, float distanceInInches);
	virtual ~TravelDistance();
	
	virtual void Initialize();
	virtual void Execute();
	virtual bool IsFinished();
	virtual void End();
	virtual void Interrupted();
	
private:
	float m_distanceInInches;
	BaseDrive *m_drive;
};

class RotateRobot : public Command
{
public:
	RotateRobot(BaseDrive *drive, float degrees);
	virtual ~RotateRobot();
	
	virtual void Initialize();
	virtual void Execute();
	virtual bool IsFinished();
	virtual void End();
	virtual void Interrupted();
	
private:
	float m_degrees;
	BaseDrive *m_drive;
};

#endif
