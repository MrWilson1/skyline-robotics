#include "ClimberCommands.h"

ClimberCommand::ExtendArm::ExtendArm(Extender::Base *climberExtender) :
		SimpleCommand("ClimberCommand::ExtendArm", false) {
	m_climberExtender = climberExtender;
	Requires(m_climberExtender);
}

ClimberCommand::ExtendArm::~ExtendArm() {
	// empty
}

void ClimberCommand::ExtendArm::Execute() {
	m_climberExtender->PullPins();
}


ClimberCommand::HookOnToRung::HookOnToRung(
		BasicArmCommand::Arm arm, 
		double heightOfRobot, 
		double distanceBetweenShoulderAndRung) :
		CommandGroup ("ClimberCommand::HookOnToRung"){
	AddSequential(new BasicArmCommand::SetPolar(arm, 0, heightOfRobot + distanceBetweenShoulderAndRung ));
	AddSequential (new BasicArmCommand::SetCartesian(arm, arm.GetX() - heightOfRobot, arm.GetY() - distanceBetweenShoulderAndRung)); // this doesn't work
}

ClimberCommand::HookOnToRung::~HookOnToRung() {
	// empty
}


ClimberCommand::PullRobotUp::PullRobotUp(BasicArmCommand::Arm arm, double magnitude) : 
		CommandGroup("ClimberCommand::PullRobotUp") {
	AddSequential(new BasicArmCommand::SetMagnitude(arm, magnitude));
}

ClimberCommand::PullRobotUp::PullRobotUp(BasicArmCommand::Arm arm) : 
		CommandGroup("PullRobotUp") {
	AddSequential(new BasicArmCommand::SetMagnitude(arm, 5.0));
}

ClimberCommand::PullRobotUp::~PullRobotUp() {
	//empty
}


ClimberCommand::DisengageArm::DisengageArm(BasicArmCommand::Arm arm) :
		CommandGroup("DisengageArm"),
		m_xDisplacement(-5.0),
		m_yDisplacement(8.0) {
	AddSequential(new BasicArmCommand::SetCartesian(arm, arm.GetX() + m_xDisplacement, arm.GetY() + m_yDisplacement));
}

ClimberCommand::DisengageArm::~DisengageArm() {
	// empty
}


ClimberCommand::ClimbLevel::ClimbLevel(
		BasicArmCommand::Arm arm, 
		double heightOfRobot, 
		double distanceBetweenShoulderAndRung) : 
		CommandGroup("ClimberCommand::ClimbLevel"){
	AddSequential(new DisengageArm(arm));
	AddSequential(new HookOnToRung(arm, heightOfRobot, distanceBetweenShoulderAndRung));
	AddSequential(new PullRobotUp(arm));
}

ClimberCommand::ClimbLevel::~ClimbLevel() {
	// empty
}



ClimberCommand::ClimbPyramid::ClimbPyramid(
		BasicArmCommand::Arm arm, 
		Extender::Base *extender, 
		double heightOfRobot, 
		double distanceBetweenShoulderAndRung) :
		CommandGroup("ClimberCommand::ClimbPyramid") {
	AddSequential(new ExtendArm(extender));
	AddSequential(new ClimbLevel(arm, heightOfRobot, distanceBetweenShoulderAndRung));
	AddSequential(new ClimbLevel(arm, heightOfRobot, distanceBetweenShoulderAndRung));
}

ClimberCommand::ClimbPyramid::~ClimbPyramid() {
	// empty
}



ClimberCommand::ControlArmManually::ControlArmManually(Joint::Base *elbow, Joint::Base *shoulder, Axis *elbowAxis, Axis *shoulderAxis) :
		SimpleCommand("ControlArmManually", true) {
	m_elbow = elbow;
	m_shoulder = shoulder;
	m_elbowAxis = elbowAxis;
	m_shoulderAxis = shoulderAxis;
	Requires(m_elbow);
	Requires(m_shoulder);
}

ClimberCommand::ControlArmManually::~ControlArmManually() {
	// empty
}

void ClimberCommand::ControlArmManually::Execute() {
	m_elbow->SetSpeed(m_elbowAxis->Get());
	m_shoulder->SetSpeed(m_shoulderAxis->Get());
}
