#ifndef AUGMENTED_TERMINATOR_ROBOT_PROFILE_H
#define AUGMENTED_TERMINATOR_ROBOT_PROFILE_H

#include "WPILib.h"

#include "../Misc/Ports.h"
#include "BaseRobotProfile.h"

#include "../Subsystems/Drive/DriveSubsystem.h"
#include "../Subsystems/Components/DriveComponents.h"
#include "../Subsystems/Experimental.h"
#include "../Subsystems/Shooter/FrisbeeTurret.h"
#include "../Subsystems/Shooter/FrisbeeLoader.h"
#include "../Subsystems/Shooter/FrisbeeShooter.h"
#include "../Subsystems/Shooter/FrisbeeAimer.h"
#include "../Subsystems/Climber/ClimberWinch.h"

#include "../Commands/DriveCommands.h"
#include "../Commands/ShooterCommands.h"
#include "../Commands/WinchCommands.h"
#include "../Commands/TestCommands.h"

#include "../Subsystems/Controllers/Axis.h"
#include "../Subsystems/Controllers/XboxController.h"

#include "../OperatorInterfaces/SimpleOI.h"
#include "../OperatorInterfaces/CompetitionOI.h"

/**
 * A profile for the Terminator robot.
 */
class AugmentedTerminatorRobotProfile : public BaseRobotProfile {
private:
	// ### Basic Hardware Objects
	// Drive
	SpeedController *m_leftFrontMotor;
	SpeedController *m_leftBackMotor;
	SpeedController *m_rightFrontMotor;
	SpeedController *m_rightBackMotor;
	Tread *m_leftTread;
	Tread *m_rightTread;
	Encoder *m_leftEncoder;
	Encoder *m_rightEncoder;
	
	// Turret
	SpeedController *m_horizontalTurretMotor;
	SpeedController *m_verticalTurretMotor;
	DigitalInput *m_turretLeftSwitch;
	DigitalInput *m_turretRightSwitch;
	DigitalInput *m_turretTopSwitch;
	DigitalInput *m_turretBottomSwitch;
	Encoder *m_horizontalTurretEncoder;
	
	// Shooter
	SpeedController *m_shooterFront;
	SpeedController *m_shooterMiddle;
	SpeedController *m_shooterBack;
	Encoder *m_shooterEncoder;
	
	// Winch
	SpeedController *m_winchMotor;
	
	// Loader
	Solenoid *m_loaderSolenoid;
	
	// Misc
	Compressor *m_compressor;	
	
	// Subsystems
	Drive::Base *m_drive;
	FrisbeeLoader::Base *m_loader;
	FrisbeeAimer::Base *m_aimer;
	FrisbeeTurret::Base *m_horizontalTurret;
	//FrisbeeTurret::Base *m_verticalTurret;
	TestEncoder *m_horizontalTurretTestEncoder;
	FrisbeeShooter::Base *m_shooter;
	Winch::Base *m_winch;
	TestEncoder *m_leftTestEncoder;
	TestEncoder *m_rightTestEncoder;
	
	// Commands and buttons
	XboxSingleJoystickOI *m_oi;
	XboxController *m_xbox;
	Joystick *m_joystick;
	
	
public:
	AugmentedTerminatorRobotProfile();
	~AugmentedTerminatorRobotProfile();
	
	void CreateBasicHardwareObjects();
	void CreateSubsystems();
	void CreateOI();
	
	void RobotInit();
	void AutonomousInit();
	void TeleopInit();
	void TeleopPeriodic();
};

#endif
