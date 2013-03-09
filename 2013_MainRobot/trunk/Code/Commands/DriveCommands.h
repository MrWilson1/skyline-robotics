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
	TankDriveCommand(Drive::Base *drive, Axis *leftAxis, Axis *rightAxis);
	~TankDriveCommand();
	/**
	 * Grabs the appropriate tank values from the OI and 
	 * displays them on the SmartDashboard.
	 */
	void Execute();
	
private:
	Drive::Base *m_drive;
	Axis *m_leftAxis;
	Axis *m_rightAxis;
};

/**
 * Issues a perpetually running command to drive in arcade mode.
 */
class ArcadeDriveCommand : public SimpleCommand {
public:
	ArcadeDriveCommand(Drive::Base *drive, Axis *magnitudeAxis, Axis *rotateAxis);
	~ArcadeDriveCommand();
	void Execute();
	
private:
	Drive::Base *m_drive;
	Axis *m_magnitudeAxis;
	Axis *m_rotateAxis;
};


class TravelStraightManualCommand : public SimpleCommand {
public:
	TravelStraightManualCommand(Drive::Base *drive, Axis *axis);
	~TravelStraightManualCommand();
	void Execute();
	
private:
	Drive::Base *m_drive;
	Axis *m_axis;
};

class RefreshPidCommand : public SimpleCommand {
public:
	RefreshPidCommand(Drive::IPid *drive, Encoder::PIDSourceParameter pidSource);
	~RefreshPidCommand();
	void Execute();
private:
	Drive::IPid *m_drive;
	Encoder::PIDSourceParameter m_pidSource;
};


class TravelDistanceCommand : public Command
{
public:
	TravelDistanceCommand(Drive::Base *drive, float distance, Tools::Units unit);
	TravelDistanceCommand(Drive::Base *drive, float distanceInInches);
	~TravelDistanceCommand();
	
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
	
private:
	float m_distanceInInches;
	Drive::Base *m_drive;
};


class RotateRobotCommand : public Command
{
public:
	RotateRobotCommand(Drive::Base *drive, float degrees);
	~RotateRobotCommand();
	
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
	
private:
	float m_degrees;
	Drive::Base *m_drive;
};


class ToggleTransmissionCommand : public SimpleCommand {
public:
	ToggleTransmissionCommand(DriveTransmission::Base *transmission);
	~ToggleTransmissionCommand();
	void Execute();
	
private:
	DriveTransmission::Base *m_transmission;
};

class ToggleTransmissionSwapPidCommand : public SimpleCommand {
public:
	ToggleTransmissionSwapPidCommand(DriveTransmission::Base *transmission, Drive::IPid *drive, 
			Drive::DrivePid lowGearRate, Drive::DrivePid lowGearDistance,
			Drive::DrivePid highGearRate, Drive::DrivePid highGearDistance);
	~ToggleTransmissionSwapPidCommand();
	void Execute();
	
private:
	DriveTransmission::Base *m_transmission;
	Drive::IPid *m_drive;
	Drive::DrivePid m_lowGearRate;
	Drive::DrivePid m_lowGearDistance;
	Drive::DrivePid m_highGearRate;
	Drive::DrivePid m_highGearDistance;
};

#endif
