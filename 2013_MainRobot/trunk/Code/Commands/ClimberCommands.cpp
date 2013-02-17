#include "ClimberCommands.h"

PullRobotUpCommand::PullRobotUpCommand(ArmData arm, double magnitude) : 
		CommandGroup("PullRobotUp") {
	AddSequential(new SetMagnitudeCommand(arm, magnitude));
}

PullRobotUpCommand::PullRobotUpCommand(ArmData arm) : 
		CommandGroup("PullRobotUp") {
	AddSequential(new SetMagnitudeCommand(arm, 5.0));
}

PullRobotUpCommand::~PullRobotUpCommand() {
	//empty
}

ExtendArmCommand::ExtendArmCommand(BaseClimberExtender *climberExtender) :
		Command("ExtendArm") {
	m_climberExtender = climberExtender;
	Requires(climberExtender);
}

ExtendArmCommand::~ExtendArmCommand() {
	// empty
}

void ExtendArmCommand::Initialize() {
	// empty
}

void ExtendArmCommand::Execute() {
	m_climberExtender->PullPins();
}

bool ExtendArmCommand::IsFinished() {
	return true;
}

void ExtendArmCommand::End() {
	// empty
}

void ExtendArmCommand::Interrupted() {
	// empty
}

DisengageArmCommand::DisengageArmCommand(ArmData arm) :
		CommandGroup("DisengageArm"),
		m_xDisplacement(-5.0),
		m_yDisplacement(8.0) {
	AddSequential(new SetCartesianCommand(arm, Arm::GetX(arm) + m_xDisplacement, Arm::GetY(arm) + m_yDisplacement));
}

DisengageArmCommand::~DisengageArmCommand() {
	// empty
}


HookAndDisengageCommand::HookAndDisengageCommand(ArmData arm, double heightOfRobot, double distanceBetweenShoulderAndRung) :
		CommandGroup("HookAndDisengage") {
	AddSequential(new HookOnToRungCommand(arm, heightOfRobot, distanceBetweenShoulderAndRung));
	AddSequential(new DisengageArmCommand(arm));
}

HookAndDisengageCommand::~HookAndDisengageCommand() {
	// empty
}


HookOnToRungCommand::HookOnToRungCommand (ArmData arm, double heightOfRobot, double distanceBetweenShoulderAndRung) :
		CommandGroup ("HookOnToRung"){
	AddSequential(new SetPolarCommand(arm, 0, heightOfRobot + distanceBetweenShoulderAndRung ));
	AddSequential (new SetCartesianCommand(arm, Arm::GetX(arm) - heightOfRobot, Arm::GetY(arm) - distanceBetweenShoulderAndRung)); // this doesn't work
}

HookOnToRungCommand::~HookOnToRungCommand() {
	// empty
}


ControlArmManuallyCommand::ControlArmManuallyCommand(BaseJoint *elbow, BaseJoint *shoulder, Axis *elbowAxis, Axis *shoulderAxis) :
		Command("ControlArmManually") {
	m_elbow = elbow;
	m_shoulder = shoulder;
	m_elbowAxis = elbowAxis;
	m_shoulderAxis = shoulderAxis;
	Requires(elbow);
	Requires(shoulder);
}

ControlArmManuallyCommand::~ControlArmManuallyCommand() {
	// empty
}

void ControlArmManuallyCommand::Initialize() {
	// empty;
}

void ControlArmManuallyCommand::Execute() {
	m_elbow->SetSpeed(m_elbowAxis->Get());
	m_shoulder->SetSpeed(m_shoulderAxis->Get());
}

bool ControlArmManuallyCommand::IsFinished() {
	return false;
}

void ControlArmManuallyCommand::End() {
	// Empty
}

void ControlArmManuallyCommand::Interrupted() {
	// Empty
}

ClimbPyramidCommand::ClimbPyramidCommand(ArmData arm, BaseClimberExtender *extender, double heightOfRobot, double distanceBetweenShoulderAndRung) {
	AddSequential(new ExtendArmCommand(extender));
	AddSequential(new ClimbLevelCommand(arm, heightOfRobot, distanceBetweenShoulderAndRung));
	AddSequential(new ClimbLevelCommand(arm, heightOfRobot, distanceBetweenShoulderAndRung));
}

ClimbPyramidCommand::~ClimbPyramidCommand() {
	// empty
}


ClimbLevelCommand::ClimbLevelCommand(ArmData arm, double heightOfRobot, double distanceBetweenShoulderAndRung) {
	AddSequential(new DisengageArmCommand(arm));
	AddSequential(new HookOnToRungCommand(arm, heightOfRobot, distanceBetweenShoulderAndRung));
	AddSequential(new PullRobotUpCommand(arm));
}

ClimbLevelCommand::~ClimbLevelCommand() {
	// empty
}
