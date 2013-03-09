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
#include "../Subsystems/Shooter/FrisbeeAimer.h"
#include "../Subsystems/Drive/DriveSubsystem.h"

#include "../Misc/Tools.h"

namespace AutonomousCommand {
/*
 * The different positions the robot can be in during Autonomous mode.
 */
enum Positions{
	kNearLeftCorner,
	kNearRightCorner,
	kFarLeftCorner,
	kFarRightCorner
};

/*
 * Command to move the robot to the center line.
 */
class MoveToCenterLine : public CommandGroup {
public:
	MoveToCenterLine(BaseDrive *drive, AutonomousCommand::Positions positions);
	~MoveToCenterLine();
	
private:
	static const float kGapFromPyramidToCenter = 108;
	static const float kGapFromPyramidToSide = 115;
	static const float kPyramidLength = 94;
	static const float kGapFromPyramidToFront = 122;
};

/*
 * Command to have the robot shoot and then move to the center line.
 */
class ShootAndGoToCenterLine : public CommandGroup {
public:
	ShootAndGoToCenterLine(
			BaseDrive *drive,
			BaseFrisbeeLoader *loader, 
			BaseFrisbeeAimer *aimer, 
			BaseAxisFrisbeeTurret *horizontalTurret, 
			BaseAxisFrisbeeTurret *verticalTurret,
			BaseFrisbeeShooter *shooter,
			Tracking::TargetType target,
			AutonomousCommand::Positions position);
	~ShootAndGoToCenterLine();
};
}

/**
 * \}
 */


#endif
