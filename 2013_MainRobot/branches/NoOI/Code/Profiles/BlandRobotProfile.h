#ifndef BLAND_ROBOT_PROFILE_H
#define BLAND_ROBOT_PROFILE_H

#include "WPILib.h"

#include "../Misc/Ports.h"
#include "BaseRobotProfile.h"


#include "../Subsystems/Drive/DriveSubsystem.h"
#include "../OperatorInterfaces/Xbox.h"

#include "../Commands/DriveCommands.h"

#include "../Subsystems/Controllers/Axis.h"


/**
 * A profile for a generic robot that simply drives around.
 */
class BlandRobotProfile : public BaseRobotProfile {
private:
	// Basic Hardware Objects
	RobotDrive *m_robotDrive;
	XboxController *m_xbox;
	
	// Subsystems
	BaseDrive *m_drive;
	
public:
	BlandRobotProfile();
	~BlandRobotProfile();
	
	void CreateBasicHardwareObjects();
	void CreateSubsystems();
	void CreateOI();
	
	void RobotInit();
	void AutonomousInit();
	void TeleopInit();
};

#endif
