#ifndef DRIVE_COMMANDS_H
#define DRIVE_COMMANDS_H

#include "WPILib.h"
#include "..\Misc\Tools.h"
#include "..\OperatorInterfaces\BaseOI.h"

#include "..\Subsystems\DriveSubsystem.h"

/**
 * \brief Issues a perpetually running command to drive in tank mode.
 */
class TankDriveCommand : public Command {
public:
	TankDriveCommand(BaseDrive *drive, BaseOI *OI);
	~TankDriveCommand();
	void Initialize();
	/**
	 * Grabs the appropriate tank values from the OI and 
	 * displays them on the SmartDashboard.
	 */
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
	
private:
	BaseDrive *m_drive;
	BaseOI *m_OI;
};

/**
 * Issues a perpetually running command to drive in arcade mode.
 */
class ArcadeDriveCommand : public Command {
public:
	ArcadeDriveCommand(BaseDrive *drive, BaseOI *OI);
	~ArcadeDriveCommand();
	
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
	
private:
	BaseDrive *m_drive;
	BaseOI *m_OI;
};

/**
 * An experimental command to make the wheels spin
 * in opposing directions.
 * 
 * This class is meant to test using commands.
 */
class Spasm : public Command
{
public:
	Spasm(BaseDrive *drive);
	~Spasm();
	
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
		
private:
	BaseDrive *m_drive;
	int counter;
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
