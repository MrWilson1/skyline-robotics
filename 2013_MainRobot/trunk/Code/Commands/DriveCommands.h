#ifndef DRIVE_COMMANDS_H
#define DRIVE_COMMANDS_H

#include "WPILib.h"
#include "../Misc/Tools.h"
#include "BaseCommand.h"

#include "../Subsystems/Drive/DriveSubsystem.h"
#include "../Subsystems/Drive/DriveTransmission.h"
#include "../Subsystems/Controllers/Axis.h"

namespace DriveCommand {

/**
 * \brief Issues a perpetually running command to drive in tank mode.
 */
class TankDrive : public SimpleCommand {
public:
	TankDrive(Drive::Base *drive, Axis *leftAxis, Axis *rightAxis);
	~TankDrive();
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
class ArcadeDrive : public SimpleCommand {
public:
	ArcadeDrive(Drive::Base *drive, Axis *magnitudeAxis, Axis *rotateAxis);
	~ArcadeDrive();
	void Execute();
	
private:
	Drive::Base *m_drive;
	Axis *m_magnitudeAxis;
	Axis *m_rotateAxis;
};


class TravelStraightManual : public SimpleCommand {
public:
	TravelStraightManual(Drive::Base *drive, Axis *axis);
	~TravelStraightManual();
	void Execute();
	
private:
	Drive::Base *m_drive;
	Axis *m_axis;
};

class RefreshPid : public SimpleCommand {
public:
	RefreshPid(Drive::IPid *drive, Encoder::PIDSourceParameter pidSource);
	~RefreshPid();
	void Execute();
private:
	Drive::IPid *m_drive;
	Encoder::PIDSourceParameter m_pidSource;
};


class TravelDistance : public Command
{
public:
	TravelDistance(Drive::Base *drive, float distance, Tools::Units unit);
	TravelDistance(Drive::Base *drive, float distanceInInches);
	~TravelDistance();
	
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
	
private:
	float m_distanceInInches;
	Drive::Base *m_drive;
};


class RotateRobot : public Command
{
public:
	RotateRobot(Drive::Base *drive, float degrees);
	~RotateRobot();
	
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
	
private:
	float m_degrees;
	Drive::Base *m_drive;
};


class ToggleTransmission : public SimpleCommand {
public:
	ToggleTransmission(DriveTransmission::Base *transmission);
	~ToggleTransmission();
	void Execute();
	
private:
	DriveTransmission::Base *m_transmission;
};

class ToggleTransmissionSwapPid : public SimpleCommand {
public:
	ToggleTransmissionSwapPid(DriveTransmission::Base *transmission, Drive::IPid *drive, 
			Drive::DrivePid lowGearRate, Drive::DrivePid lowGearDistance,
			Drive::DrivePid highGearRate, Drive::DrivePid highGearDistance);
	~ToggleTransmissionSwapPid();
	void Execute();
	
private:
	DriveTransmission::Base *m_transmission;
	Drive::IPid *m_drive;
	Drive::DrivePid m_lowGearRate;
	Drive::DrivePid m_lowGearDistance;
	Drive::DrivePid m_highGearRate;
	Drive::DrivePid m_highGearDistance;
};

}

#endif
