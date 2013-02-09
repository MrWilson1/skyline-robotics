#include "ClimberCommands.h"

PullRobotUpCommand::PullRobotUpCommand(BaseClimberArm *arm) : 
		Command ("PullRobotUp"){
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
	m_arm->SetMagnitude(5);
}

void PullRobotUpCommand::IsFinished() {
	return (m_arm->GetMagnitude() <= 5);
}

void PullRobotUpCommand::End(){
	//empty
}

void PullRobotUpCommand::Interrupted(){
	//empty
}
