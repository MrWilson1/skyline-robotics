#ifndef BLAND_ROBOT_PROFILE_H
#define BLAND_ROBOT_PROFILE_H

#include "WPILib.h"

#include "../Misc/Ports.h"
#include "BaseRobotProfile.h"


#include "../Subsystems/Drive/DriveSubsystem.h"

#include "../Commands/DriveCommands.h"

#include "../Subsystems/Controllers/Axis.h"
#include "../Subsystems/Controllers/XboxController.h"

#include "../OperatorInterfaces/SimpleOI.h"

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
	
	SimpleOI *m_oi;
	
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
