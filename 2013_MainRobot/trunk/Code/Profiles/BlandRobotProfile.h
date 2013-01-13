#ifndef BLAND_ROBOT_PROFILE_H
#define BLAND_ROBOT_PROFILE_H

#include "WPILib.h"
#include "../Tools/Ports.h"
#include "../Input/XboxOI.h"
#include "../Input/Xbox.h"

class BlandRobotProfile : public IterativeRobot {
private:
	LiveWindow *lw;
	RobotDrive *m_robotDrive;
	XboxController *m_xbox;
	
	BaseDrive *m_drive; 
	
	BaseOI *m_OI;
	
public:
	BlandRobotProfile();
	virtual ~BlandRobotProfile();
	
	virtual void RobotInit();
	virtual void AutonomousInit();
	virtual void AutonomousPeriodic();
	virtual void TeleopInit();
	virtual void TeleopPeriodic();
	virtual void TestPeriodic();
};

#endif
