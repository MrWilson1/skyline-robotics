#include "BaseRobotProfile.h"

BaseRobotProfile::BaseRobotProfile(const char *name) :
		IterativeRobot(),
		m_name(name) {
	m_liveWindow = LiveWindow::GetInstance();
}

BaseRobotProfile::~BaseRobotProfile() {
	// empty
}

/*std::string BaseRobotProfile::GetName() {
	return m_name;
}*/

/**
 * Runs all activated commands during Autonomous, 
 * and automatically removes finished ones.
 */
void BaseRobotProfile::AutonomousPeriodic() {
	Scheduler::GetInstance()->Run();
}

/**
 * Runs all activated commands during Teleop, 
 * and automatically removes finished ones.
 */
void BaseRobotProfile::TeleopPeriodic() {
	Scheduler::GetInstance()->Run();
}

/**
 * Runs all test and dignostic LiveWindow stuff
 * during Test mode.
 */
void BaseRobotProfile::TestPeriodic() {
	m_liveWindow->Run();
}

std::string BaseRobotProfile::GetName() {
	return m_name;
}
