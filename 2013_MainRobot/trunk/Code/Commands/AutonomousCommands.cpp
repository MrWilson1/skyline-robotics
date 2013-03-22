#include "AutonomousCommands.h"



AutonomousCommand::DoNothing::DoNothing() :
		SimpleCommand("AutonomousCommand::DoNothing", true) {
	// empty
}


AutonomousCommand::DoNothing::~DoNothing() {
	// empty
}

void AutonomousCommand::DoNothing::Execute() {
	// empty
}



AutonomousCommand::LoadNFrisbees::LoadNFrisbees(
		FrisbeeLoader::Base *loader, 
		unsigned int number) :
		CommandGroup("AutonomousCommand::LoadNFrisbees") {
	for (unsigned int i = 0; i < number; i++) {
		Wait(2);
		AddSequential(new ShooterCommand::LoadFrisbee(loader));
	}
}
	
AutonomousCommand::LoadNFrisbees::~LoadNFrisbees() {
	// empty
}




AutonomousCommand::FireNFrisbees::FireNFrisbees(
		FrisbeeLoader::Base *loader, 
		FrisbeeShooter::Base *shooter, 
		unsigned int number) :
		CommandGroup("AutonomousCommand::FireNFrisbees") {
	AddParallel(new ShooterCommand::FireFrisbee(shooter));
	AddParallel(new AutonomousCommand::LoadNFrisbees(loader, 3));
}

AutonomousCommand::FireNFrisbees::~FireNFrisbees() {
	// empty
}













AutonomousCommand::MoveToCenterLine::MoveToCenterLine(Drive::Base *drive, AutonomousCommand::Positions positions) : 
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
	AddSequential(new DriveCommand::RotateRobot(drive, rotation));
	AddSequential(new DriveCommand::TravelDistance(drive, distance - 50));
}

AutonomousCommand::MoveToCenterLine::~MoveToCenterLine() {
	// empty
}

AutonomousCommand::ShootAndGoToCenterLine::ShootAndGoToCenterLine(
			Drive::Base *drive,
			FrisbeeLoader::Base *loader, 
			FrisbeeAimer::Base *aimer,
			FrisbeeTurret::Base *horizontalTurret, 
			FrisbeeTurret::Base *verticalTurret,
			FrisbeeShooter::Base *shooter,
			Tracking::TargetType target,
			AutonomousCommand::Positions position) :
			CommandGroup("ShootAndGoToCenterLine") {
	AddSequential(new TurretCommand::AimTurret(aimer, horizontalTurret, verticalTurret, target, 5.0), 5.0);
	AddSequential(new ShooterCommand::LoadAndFire(loader, shooter));
	AddSequential(new MoveToCenterLine(drive, position));
}

AutonomousCommand::ShootAndGoToCenterLine::~ShootAndGoToCenterLine() {
	// empty
}
