#ifndef MAIN_ROBOT_2013_PROFILE_H
#define MAIN_ROBOT_2013_PROFILE_H

#include "WPILib.h"

#include "../Misc/Ports.h"

#include "BaseRobotProfile.h"

#include "../Subsystems/Drive/DriveSubsystem.h"
#include "../Subsystems/Components/DriveComponents.h"
#include "../Subsystems/Shooter/FrisbeeTurret.h"
#include "../Subsystems/Shooter/FrisbeeLoader.h"
#include "../Subsystems/Shooter/FrisbeeShooter.h"
#include "../Subsystems/Shooter/FrisbeeAimer.h"
#include "../Subsystems/Experimental.h"

#include "../Commands/DriveCommands.h"
#include "../Commands/ShooterCommands.h"
#include "../Commands/TestCommands.h"
#include "../Commands/WinchCommands.h"
#include "../Commands/TurretCommands.h"
#include "../Commands/AutonomousCommands.h"

#include "../Subsystems/Controllers/Axis.h"
#include "../Subsystems/Controllers/XboxController.h"

#include "../OperatorInterfaces/CompetitionOI.h"

/**
 * A profile for a generic robot that simply drives around.
 */
class MainRobot2013Profile : public BaseRobotProfile {
private:
	// Basic Hardware Objects
	SpeedController *m_leftMotor;
	SpeedController *m_rightMotor;
	Tread *m_leftTread;
	Tread *m_rightTread;
	Encoder *m_leftEncoder;
	Encoder *m_rightEncoder;
	
	DoubleSolenoid *m_leftTransmissionSolenoid;
	DoubleSolenoid *m_rightTransmissionSolenoid;

	SpeedController *m_shooterFront;
	SpeedController *m_shooterMiddle;
	SpeedController *m_shooterBack;
	Encoder *m_shooterEncoder;
	Solenoid *m_loaderSolenoid;
	
	SpeedController *m_turretHorizontalMotor;
	SpeedController *m_turretVerticalMotor;
	Encoder *m_turretHorizontalEncoder;
	Encoder *m_turretVerticalEncoder;
	DigitalInput *m_turretTopSwitch;
	DigitalInput *m_turretBottomSwitch;
	
	SpeedController *m_winchMotor;
	
	Compressor *m_compressor;
	
	// Subsystems
	Drive::Base *m_drive;
	DriveTransmission::Base *m_transmission;
	
	FrisbeeShooter::Base *m_shooter;
	FrisbeeLoader::Base *m_loader;
	FrisbeeAimer::Base *m_aimer;
	
	FrisbeeTurret::Base *m_horizontalTurret;
	FrisbeeTurret::Base *m_verticalTurret;
	
	Winch::Base *m_winch;
	
	TestEncoder *m_leftTestEncoder;
	TestEncoder *m_rightTestEncoder;
	TestEncoder *m_horizontalTurretTestEncoder;
	TestEncoder *m_verticalTurretTestEncoder;
	TestEncoder *m_shooterTestEncoder;
	
	// OI
	XboxController *m_xboxDrive;
	XboxController *m_xboxShooter;
	Joystick *m_joystick;
	CompetitionOI *m_oi;
	
	// Autonomous
	SendableChooser *autoChooser;
	Command *autoCommand;
	
public:
	MainRobot2013Profile();
	~MainRobot2013Profile();
	
	void CreateBasicHardwareObjects();
	void CreateSubsystems();
	void CreateOI();
	
	void RobotInit();
	void AutonomousInit();
	void TeleopInit();
};

#endif
