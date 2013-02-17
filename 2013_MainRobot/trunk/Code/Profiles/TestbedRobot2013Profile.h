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

#include "../OperatorInterfaces/CompetitionOI.h"

#include "../Subsystems/Controllers/XboxController.h"
#include "../Subsystems/Controllers/Axis.h"


class TestbedRobot2013Profile : public BaseRobotProfile {
private:
	// Basic Hardware Objects
	SpeedController *m_leftMotor;
	SpeedController *m_rightMotor;
	Tread *m_leftTread;
	Tread *m_rightTread;
	
	XboxController *m_xbox;
	Compressor *m_compressor;
	Encoder *m_leftEncoder;
	Encoder *m_rightEncoder;
	DoubleSolenoid *m_leftTransmissionSolenoid;
	DoubleSolenoid *m_rightTransmissionSolenoid;
	
	// Subsystems
	BaseDrive *m_drive;
	TestEncoder *m_leftTestEncoder;
	TestEncoder *m_rightTestEncoder;
	BaseDriveTransmission *m_transmission;
	
	CompetitionOI *m_oi;
	
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
