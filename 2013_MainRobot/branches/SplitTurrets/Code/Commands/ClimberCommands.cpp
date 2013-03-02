#include "ClimberCommands.h"

ExtendArmCommand::ExtendArmCommand(BaseClimberExtender *climberExtender) :
		SimpleCommand("ExtendArm", false) {
	m_climberExtender = climberExtender;
	Requires(m_climberExtender);
}

ExtendArmCommand::~ExtendArmCommand() {
	// empty
}

void ExtendArmCommand::Execute() {
	m_climberExtender->PullPins();
}


HookOnToRungCommand::HookOnToRungCommand (Arm arm, double heightOfRobot, double distanceBetweenShoulderAndRung) :
		CommandGroup ("HookOnToRung"){
	AddSequential(new SetPolarCommand(arm, 0, heightOfRobot + distanceBetweenShoulderAndRung ));
	AddSequential (new SetCartesianCommand(arm, arm.GetX() - heightOfRobot, arm.GetY() - distanceBetweenShoulderAndRung)); // this doesn't work
}

HookOnToRungCommand::~HookOnToRungCommand() {
	// empty
}


PullRobotUpCommand::PullRobotUpCommand(Arm arm, double magnitude) : 
		CommandGroup("PullRobotUp") {
	AddSequential(new SetMagnitudeCommand(arm, magnitude));
}

PullRobotUpCommand::PullRobotUpCommand(Arm arm) : 
		CommandGroup("PullRobotUp") {
	AddSequential(new SetMagnitudeCommand(arm, 5.0));
}

PullRobotUpCommand::~PullRobotUpCommand() {
	//empty
}


DisengageArmCommand::DisengageArmCommand(Arm arm) :
		CommandGroup("DisengageArm"),
		m_xDisplacement(-5.0),
		m_yDisplacement(8.0) {
	AddSequential(new SetCartesianCommand(arm, arm.GetX() + m_xDisplacement, arm.GetY() + m_yDisplacement));
}

DisengageArmCommand::~DisengageArmCommand() {
	// empty
}


ClimbLevelCommand::ClimbLevelCommand(Arm arm, double heightOfRobot, double distanceBetweenShoulderAndRung) {
	AddSequential(new DisengageArmCommand(arm));
	AddSequential(new HookOnToRungCommand(arm, heightOfRobot, distanceBetweenShoulderAndRung));
	AddSequential(new PullRobotUpCommand(arm));
}

ClimbLevelCommand::~ClimbLevelCommand() {
	// empty
}


ClimbPyramidCommand::ClimbPyramidCommand(Arm arm, BaseClimberExtender *extender, double heightOfRobot, double distanceBetweenShoulderAndRung) {
	AddSequential(new ExtendArmCommand(extender));
	AddSequential(new ClimbLevelCommand(arm, heightOfRobot, distanceBetweenShoulderAndRung));
	AddSequential(new ClimbLevelCommand(arm, heightOfRobot, distanceBetweenShoulderAndRung));
}

ClimbPyramidCommand::~ClimbPyramidCommand() {
	// empty
}


ControlArmManuallyCommand::ControlArmManuallyCommand(BaseJoint *elbow, BaseJoint *shoulder, Axis *elbowAxis, Axis *shoulderAxis) :
		SimpleCommand("ControlArmManually", true) {
	m_elbow = elbow;
	m_shoulder = shoulder;
	m_elbowAxis = elbowAxis;
	m_shoulderAxis = shoulderAxis;
	Requires(m_elbow);
	Requires(m_shoulder);
}

ControlArmManuallyCommand::~ControlArmManuallyCommand() {
	// empty
}

void ControlArmManuallyCommand::Execute() {
	m_elbow->SetSpeed(m_elbowAxis->Get());
	m_shoulder->SetSpeed(m_shoulderAxis->Get());
}
