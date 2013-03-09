#ifndef BLAND_ROBOT_PROFILE_H
#define BLAND_ROBOT_PROFILE_H

#include "WPILib.h"

#include "../Misc/Ports.h"
#include "BaseRobotProfile.h"


#include "../Subsystems/Drive/DriveSubsystem.h"
#include "../Subsystems/Components/DriveComponents.h"

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
	SpeedController *m_leftFrontMotor;
	SpeedController *m_leftBackMotor;
	SpeedController *m_rightFrontMotor;
	SpeedController *m_rightBackMotor;
	Tread *m_leftTread;
	Tread *m_rightTread;
	XboxController *m_xbox;
	
	// Subsystems
	Drive::Base *m_drive;
	
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
