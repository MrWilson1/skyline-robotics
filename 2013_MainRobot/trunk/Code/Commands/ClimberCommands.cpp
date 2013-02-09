#include "ClimberCommands.h"

PullRobotUpCommand::PullRobotUpCommand(BaseClimberArm *arm) : 
		Command ("PullRobotUp"),
		m_magnitude(5.0) {
	m_arm = arm;
	Requires (m_arm); 
}

PullRobotUpCommand::PullRobotUpCommand(BaseClimberArm *arm, float magnitude) : 
		Command ("PullRobotUp"),
		m_magnitude(magnitude) {
	m_arm = arm;
	Requires (m_arm); 
}

PullRobotUpCommand::~PullRobotUpCommand() {
	//empty
}

void PullRobotUpCommand::Initialize() {
	//empty
}

void PullRobotUpCommand::Execute() {
	m_arm->SetMagnitude(m_magnitude);
}

void PullRobotUpCommand::IsFinished() {
	return (m_arm->GetMagnitude() <= m_magnitude);
}

void PullRobotUpCommand::End(){
	//empty
}

void PullRobotUpCommand::Interrupted(){
	//empty
}
