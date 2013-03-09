#include "AutonomousCommands.h"

AutonomousCommand::MoveToCenterLine::MoveToCenterLine(BaseDrive *drive, AutonomousCommand::Positions positions) : 
		CommandGroup("MoveToCenterLine") {
	float distance;
	float rotation = 180;
	switch (positions) {
	case (AutonomousCommand::kNearLeftCorner):
	case (AutonomousCommand::kNearRightCorner):
		distance = kGapFromPyramidToCenter;
		break;
	case (AutonomousCommand::kFarLeftCorner):
	case (AutonomousCommand::kFarRightCorner):
		distance = kGapFromPyramidToCenter + kPyramidLength;
		break;
	default:
		distance = kGapFromPyramidToCenter;
	}
	AddSequential(new RotateRobotCommand(drive, rotation));
	AddSequential(new TravelDistanceCommand(drive, distance - 50));
}

AutonomousCommand::MoveToCenterLine::~MoveToCenterLine() {
	// empty
}

AutonomousCommand::ShootAndGoToCenterLine::ShootAndGoToCenterLine(
			BaseDrive *drive,
			BaseFrisbeeLoader *loader, 
			BaseFrisbeeAimer *aimer, 
			BaseAxisFrisbeeTurret *horizontalTurret, 
			BaseAxisFrisbeeTurret *verticalTurret,
			BaseFrisbeeShooter *shooter,
			Tracking::TargetType target,
			AutonomousCommand::Positions position) :
			CommandGroup("ShootAndGoToCenterLine") {
	AddSequential(new AimTurretCommand(aimer, horizontalTurret, verticalTurret, target, 5.0), 5.0);
	AddSequential(new LoadAndFireCommand(loader, aimer, horizontalTurret, verticalTurret, shooter));
	AddSequential(new MoveToCenterLine(drive, position));
}

AutonomousCommand::ShootAndGoToCenterLine::~ShootAndGoToCenterLine() {
	// empty
}
