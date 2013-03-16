#ifndef DRIVE_COMMANDS_H
#define DRIVE_COMMANDS_H

#include "WPILib.h"
#include "../Misc/Tools.h"
#include "BaseCommand.h"

#include "../Subsystems/Drive/DriveSubsystem.h"
#include "../Subsystems/Drive/DriveTransmission.h"
#include "../Subsystems/Controllers/Axis.h"

/**
 * \brief Contains commands related to driving the robot, and its position on the field.
 */
namespace DriveCommand {

/**
 * \brief Issues a perpetually running command to drive in tank mode.
 * 
 * Also grabs the appropriate tank values from the OI and displays them
 * on the SmartDashboard.
 */
class TankDrive : public SimpleCommand {
public:
	TankDrive(Drive::Base *drive, BaseAxis *leftAxis, BaseAxis *rightAxis);
	~TankDrive();
	void Execute();
	
private:
	Drive::Base *m_drive;
	BaseAxis *m_leftAxis;
	BaseAxis *m_rightAxis;
};

/**
 * \brief Issues a perpetually running command to drive in arcade mode.
 */
class ArcadeDrive : public SimpleCommand {
public:
	ArcadeDrive(Drive::Base *drive, BaseAxis *magnitudeAxis, BaseAxis *rotateAxis);
	~ArcadeDrive();
	void Execute();
	
private:
	Drive::Base *m_drive;
	BaseAxis *m_magnitudeAxis;
	BaseAxis *m_rotateAxis;
};

/**
 * \brief Tells the robot to travel straight given input from an axis.
 */
class TravelStraightManual : public SimpleCommand {
public:
	TravelStraightManual(Drive::Base *drive, BaseAxis *axis);
	~TravelStraightManual();
	void Execute();
	
private:
	Drive::Base *m_drive;
	BaseAxis *m_axis;
};

/**
 * \brief Given the PID source, 
 */
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
