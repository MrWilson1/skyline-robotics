#ifndef DRIVE_COMMANDS_H
#define DRIVE_COMMANDS_H

#include "WPILib.h"
#include "../Misc/Tools.h"
#include "BaseCommand.h"

#include "../Subsystems/Drive/DriveSubsystem.h"
#include "../Subsystems/Drive/DriveTransmission.h"
#include "../Subsystems/Controllers/Axis.h"

/**
 * \brief Issues a perpetually running command to drive in tank mode.
 */
class TankDriveCommand : public SimpleCommand {
public:
	TankDriveCommand(BaseDrive *drive, Axis *leftAxis, Axis *rightAxis);
	~TankDriveCommand();
	/**
	 * Grabs the appropriate tank values from the OI and 
	 * displays them on the SmartDashboard.
	 */
	void Execute();
	
private:
	BaseDrive *m_drive;
	Axis *m_leftAxis;
	Axis *m_rightAxis;
};

/**
 * Issues a perpetually running command to drive in arcade mode.
 */
class ArcadeDriveCommand : public SimpleCommand {
public:
	ArcadeDriveCommand(BaseDrive *drive, Axis *magnitudeAxis, Axis *rotateAxis);
	~ArcadeDriveCommand();
	void Execute();
	
private:
	BaseDrive *m_drive;
	Axis *m_magnitudeAxis;
	Axis *m_rotateAxis;
};


class TravelStraightManualCommand : public SimpleCommand {
public:
	TravelStraightManualCommand(BaseDrive *drive, Axis *axis);
	~TravelStraightManualCommand();
	void Execute();
	
private:
	BaseDrive *m_drive;
	Axis *m_axis;
};

class RefreshPidCommand : public SimpleCommand {
public:
	RefreshPidCommand(IPidDrive *drive, Encoder::PIDSourceParameter pidSource);
	~RefreshPidCommand();
	void Execute();
private:
	IPidDrive *m_drive;
	Encoder::PIDSourceParameter m_pidSource;
};


class TravelDistanceCommand : public Command
{
public:
	TravelDistanceCommand(BaseDrive *drive, float distance, Tools::Units unit);
	TravelDistanceCommand(BaseDrive *drive, float distanceInInches);
	~TravelDistanceCommand();
	
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
	
private:
	float m_distanceInInches;
	BaseDrive *m_drive;
};


class RotateRobotCommand : public Command
{
public:
	RotateRobotCommand(BaseDrive *drive, float degrees);
	~RotateRobotCommand();
	
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
	
private:
	float m_degrees;
	BaseDrive *m_drive;
};


class ToggleTransmissionCommand : public SimpleCommand {
public:
	ToggleTransmissionCommand(BaseDriveTransmission *transmission);
	~ToggleTransmissionCommand();
	void Execute();
	
private:
	BaseDriveTransmission *m_transmission;
};


#endif
