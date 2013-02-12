#ifndef TESTBED_ROBOT_2013_PROFILE_H
#define TESTBED_ROBOT_2013_PROFILE_H

#include "WPILib.h"

#include "../Misc/Ports.h"
#include "BaseRobotProfile.h"

#include "../Subsystems/Drive/DriveSubsystem.h"
#include "../Subsystems/Drive/DriveTransmission.h"
#include "../Subsystems/Experimental.h"

#include "../Commands/DriveCommands.h"
#include "../Commands/TestCommands.h"

#include "../OperatorInterfaces/Xbox.h"
#include "../Subsystems/Controllers/Axis.h"


class TestbedRobot2013Profile : public BaseRobotProfile {
private:
	// Basic Hardware Objects
	RobotDrive *m_robotDrive;
	XboxController *m_xbox;
	Compressor *m_compressor;
	Encoder *m_leftEncoder;
	Encoder *m_rightEncoder;
	
	// Subsystems
	BaseDrive *m_drive;
	TestEncoder *m_leftTestEncoder;
	TestEncoder *m_rightTestEncoder;
	BaseDriveTransmission *m_transmission;
	
	// Buttons and commands
	JoystickButton *m_driveStraightButton;
	JoystickButton *m_toggleTransmissionButton;
	
public:
	TestbedRobot2013Profile();
	~TestbedRobot2013Profile();
	
	void CreateBasicHardwareObjects();
	void CreateSubsystems();
	void CreateOI();
	
	void RobotInit();
	void AutonomousInit();
	void TeleopInit();
};

#endif
