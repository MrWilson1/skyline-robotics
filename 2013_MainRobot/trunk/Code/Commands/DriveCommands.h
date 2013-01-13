#ifndef DRIVE_COMMANDS_H
#define DRIVE_COMMANDS_H

#include "WPILib.h"
#include "..\Tools\Units.h"
#include "..\Input\BaseOI.h"

#include "..\Subsystems\DriveSubsystem.h"

class TankDrive : public Command
{
public:
	TankDrive(BaseDrive *drive, BaseOI *OI);
	~TankDrive();
	virtual void Initialize();
	virtual void Execute();
	virtual bool IsFinished();
	virtual void End();
	virtual void Interrupted();
	
private:
	BaseDrive *m_drive;
	BaseOI *m_OI;
};

class ArcadeDrive : public Command
{
public:
	ArcadeDrive(BaseDrive *drive, BaseOI *OI);
	virtual ~ArcadeDrive();
	
	virtual void Initialize();
	virtual void Execute();
	virtual bool IsFinished();
	virtual void End();
	virtual void Interrupted();
	
private:
	BaseDrive *m_drive;
	BaseOI *m_OI;
};

class Spasm : public Command
{
public:
	Spasm(BaseDrive *drive);
	virtual ~Spasm();
	
	virtual void Initialize();
	virtual void Execute();
	virtual bool IsFinished();
	virtual void End();
	virtual void Interrupted();
		
private:
	BaseDrive *m_drive;
};

/*
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
*/
#endif
