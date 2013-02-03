#ifndef MAIN_ROBOT_2013_PROFILE_H
#define MAIN_ROBOT_2013_PROFILE_H

#include "WPILib.h"

#include "../Misc/Ports.h"

#include "BaseRobotProfile.h"
#include "../OperatorInterfaces/CompetitionXboxOI.h"
#include "../OperatorInterfaces/Xbox.h"

#include "../Subsystems/DriveSubsystem.h"
#include "../Subsystems/FrisbeeTurret.h"
#include "../Subsystems/FrisbeeLoader.h"
#include "../Subsystems/FrisbeeShooter.h"
#include "../Subsystems/FrisbeeAimer.h"

/**
 * A profile for a generic robot that simply drives around.
 */
class MainRobot2013Profile : public BaseRobotProfile {
private:
	// Basic Hardware Objects
	SpeedController *m_leftFront; 
	SpeedController *m_leftBack;
	SpeedController *m_rightFront;
	SpeedController *m_rightBack;
	Encoder *m_leftEncoder;
	Encoder *m_rightEncoder;
	
	SpeedController *m_turretHorizontal;
	SpeedController *m_turretVertical;
	Encoder *m_turretHorizontalEncoder;
	Encoder *m_turretVerticalEncoder;
	
	SpeedController *m_shooterFront;
	SpeedController *m_shooterMiddle;
	SpeedController *m_shooterLast;
	
	XboxController *m_xbox;
	
	// Subsystems
	BaseDrive *m_drive; 
	BaseFrisbeeLoader *m_loader;
	BaseFrisbeeAimer *m_aimer;
	BaseFrisbeeTurret *m_turret;
	BaseFrisbeeShooter *m_shooter;
	
	// Operator Interfaces
	BaseOI *m_OI;
	
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
