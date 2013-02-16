#include "ClimberCommands.h"

PullRobotUpCommand::PullRobotUpCommand(BaseClimberArm *arm, float magnitude) : 
		Command ("PullRobotUp"),
		m_magnitude(magnitude) {
	m_arm = arm;
	Requires(m_arm); 
}

PullRobotUpCommand::PullRobotUpCommand(BaseClimberArm *arm) : 
		Command ("PullRobotUp"),
		m_magnitude(5.0) {
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

bool PullRobotUpCommand::IsFinished() {
	return (m_arm->GetMagnitude() <= m_magnitude);
}

void PullRobotUpCommand::End(){
	//empty
}

void PullRobotUpCommand::Interrupted(){
	//empty
}

ExtendArmCommand::ExtendArmCommand(BaseClimberExtender *climberExtender) :
		Command("ExtendArm") {
	m_climberExtender = climberExtender;
}

void ExtendArmCommand::Initialize() {
	// empty
}

void ExtendArmCommand::Execute() {
	m_climberExtender->PullPins();
}

void ExtendArmCommand::End() {
	// empty
}

void ExtendArmCommand::Interrupted() {
	// empty
}

MoveArmCartesianCommand::MoveArmCartesianCommand(BaseClimberArm *arm, double newX, double newY) :
			Command("MoveArmCartesian"),
			m_newX(newX),
			m_newY(newY),
			range(1.0) {
	m_arm = arm;
	Requires(m_arm);
}

MoveArmCartesianCommand::~MoveArmCartesianCommand() {
	// empty
}

void MoveArmCartesianCommand::Initialize() {
	// empty
}

void MoveArmCartesianCommand::Execute() {
	m_arm->SetCartesian(m_newX, m_newY);
}

bool MoveArmCartesianCommand::IsFinished() {
	return (Tools::IsWithinRange(m_arm->GetX(), m_newX, range) && Tools::IsWithinRange(m_arm->GetY(), m_newY, range));
}

void MoveArmCartesianCommand::End() {
	// empty
}

void MoveArmCartesianCommand::Interrupted() {
	// empty
}

MoveArmPolarCommand::MoveArmPolarCommand(BaseClimberArm *arm, double newAngle, double newMagnitude) :
		Command("MoveArmPolar"),
		m_newAngle(newAngle),
		m_newMagnitude(newMagnitude),
		m_angleRange(3.0),
		m_magnitudeRange(3.0) {
	m_arm = arm;
	Requires(m_arm);
}

MoveArmPolarCommand::~MoveArmPolarCommand() {
	// empty
}

void MoveArmPolarCommand::Initialize() {
	// empty
}

void MoveArmPolarCommand::Execute() {
	m_arm->SetPolar(m_newAngle, m_newMagnitude);
}

bool MoveArmPolarCommand::IsFinished() {
	return (Tools::IsWithinRange(m_arm->GetAngle(), m_newAngle, m_angleRange) && Tools::IsWithinRange(m_arm->GetMagnitude(), m_newMagnitude, m_magnitudeRange));
}

void MoveArmPolarCommand::End() {
	// empty
}

void MoveArmPolarCommand::Interrupted() {
	// empty
}

DisengageArmCommand::DisengageArmCommand(BaseClimberArm *arm) :
		CommandGroup("DisengageArm"),
		m_xDisplacement(-5.0),
		m_yDisplacement(8.0) {
	AddSequential(new MoveArmCartesianCommand(arm, arm->GetX() + m_xDisplacement, arm->GetY() + m_yDisplacement));
}

DisengageArmCommand::~DisengageArmCommand() {
	// empty
}


HookOnToRungCommand::HookOnToRungCommand (BaseClimberArm *arm, double x, double y ) :
		CommandGroup ("HookOnToRung"){
	AddSequential(new MoveArmPolarCommand(arm,0, x + y ));
	AddSequential (new MoveArmCartesianCommand (arm, m_arm->GetX(), sqrt(2)*x));
}

HookOnToRungCommand::~HookOnToRungCommand (){
	//empty
}	

