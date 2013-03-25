#ifndef AUTONOMOUS_COMMANDS_H
#define AUTONOMOUS_COMMANDS_H
/**
 * \file AutonomousCommands.h
 * 
 * \addtogroup commands
 * \{
 */

#include "WPILib.h"
#include "DriveCommands.h"
#include "ShooterCommands.h"
#include "TurretCommands.h"
#include "../Subsystems/Shooter/FrisbeeLoader.h"
#include "../Subsystems/Shooter/FrisbeeAimer.h"
#include "../Subsystems/Drive/DriveSubsystem.h"

#include "../Misc/Tools.h"

/**
 * \brief Short, scripted commands meant to run during autonomous mode.
 */
namespace AutonomousCommand {
/**
 * \brief The different positions the robot can be in during Autonomous mode.
 */
enum Positions{
	kNearLeftCorner,
	kNearRightCorner,
	kFarLeftCorner,
	kFarRightCorner
};


class DoNothing : public SimpleCommand {
public:
	DoNothing();
	~DoNothing();
	void Execute();
};


class LoadNFrisbees : public CommandGroup {
public:
	LoadNFrisbees(FrisbeeLoader::Base *loader, unsigned int number);
	~LoadNFrisbees();
};



class FireNFrisbees : public CommandGroup {
public:
	FireNFrisbees(FrisbeeLoader::Base *loader, FrisbeeShooter::Base *shooter, unsigned int number);
	~FireNFrisbees();
};


class AimTurretFireNFrisbees : public CommandGroup {
public:
	AimTurretFireNFrisbees(
			FrisbeeLoader::Base *loader, 
			FrisbeeTurret::Base *turret,
			TurretPosition::Base *position,
			FrisbeeShooter::Base *shooter,
			unsigned int number);
	~AimTurretFireNFrisbees();
};


/**
 * \brief Command to move the robot to the center line.
 */
class MoveToCenterLine : public CommandGroup {
public:
	MoveToCenterLine(Drive::Base *drive, AutonomousCommand::Positions positions);
	~MoveToCenterLine();
	
private:
	static const float kGapFromPyramidToCenter = 108;
	static const float kGapFromPyramidToSide = 115;
	static const float kPyramidLength = 94;
	static const float kGapFromPyramidToFront = 122;
};

/**
 * \brief Command to have the robot shoot and then move to the center line.
 */
class ShootAndGoToCenterLine : public CommandGroup {
public:
	ShootAndGoToCenterLine(
			Drive::Base *drive,
			FrisbeeLoader::Base *loader, 
			FrisbeeAimer::Base *aimer,
			FrisbeeTurret::Base *horizontalTurret, 
			FrisbeeTurret::Base *verticalTurret,
			FrisbeeShooter::Base *shooter,
			Tracking::TargetType target,
			AutonomousCommand::Positions position);
	~ShootAndGoToCenterLine();
};
}

/**
 * \}
 */

#endif
