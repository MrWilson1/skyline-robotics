#include "AutonomousCommands.h"

MoveToCenterLine::MoveToCenterLine(Drive::Base *drive, Autonomous::Positions positions) : 
		CommandGroup("MoveToCenterLine") {
	float distance;
	float rotation = 180;
	switch (positions) {
	case (Autonomous::kNearLeftCorner):
	case (Autonomous::kNearRightCorner):
		distance = kGapFromPyramidToCenter;
		break;
	case (Autonomous::kFarLeftCorner):
	case (Autonomous::kFarRightCorner):
		distance = kGapFromPyramidToCenter + kPyramidLength;
		break;
	default:
		distance = kGapFromPyramidToCenter;
	}
	AddSequential(new RotateRobotCommand(drive, rotation));
	AddSequential(new TravelDistanceCommand(drive, distance - 50));
}

MoveToCenterLine::~MoveToCenterLine() {
	// empty
}

ShootAndGoToCenterLine::ShootAndGoToCenterLine(
			Drive::Base *drive,
			BaseFrisbeeLoader *loader, 
			BaseFrisbeeAimer *aimer, 
			FrisbeeTurret::Base *horizontalTurret, 
			FrisbeeTurret::Base *verticalTurret,
			BaseFrisbeeShooter *shooter,
			Tracking::TargetType target,
			Autonomous::Positions position) :
			CommandGroup("ShootAndGoToCenterLine") {
	AddSequential(new AimTurretCommand(aimer, horizontalTurret, verticalTurret, target, 5.0), 5.0);
	AddSequential(new LoadAndFireFrisbeeCommand(loader, shooter));
	AddSequential(new MoveToCenterLine(drive, position));
}

ShootAndGoToCenterLine::~ShootAndGoToCenterLine() {
	// empty
}