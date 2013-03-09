#ifndef AUTONOMOUS_COMMANDS_H
#define AUTONOMOUS_COMMANDS_H

#include "WPILib.h"
#include "DriveCommands.h"
#include "ShooterCommands.h"
#include "../Subsystems/Shooter/FrisbeeAimer.h"
#include "../Subsystems/Drive/DriveSubsystem.h"

#include "../Misc/Tools.h"

namespace Autonomous {
enum Positions{
	kNearLeftCorner,
	kNearRightCorner,
	kFarLeftCorner,
	kFarRightCorner
};
}

class MoveToCenterLine : public CommandGroup {
public:
	MoveToCenterLine(BaseDrive *drive, Autonomous::Positions positions);
	~MoveToCenterLine();
	
private:
	static const float kGapFromPyramidToCenter = 108;
	static const float kGapFromPyramidToSide = 115;
	static const float kPyramidLength = 94;
	static const float kGapFromPyramidToFront = 122;
};

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
			Autonomous::Positions position);
	~ShootAndGoToCenterLine();
};


#endif
