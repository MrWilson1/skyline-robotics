#include "BasicArmCommands.h"

Arm::Arm(Joint::BaseSmart *elbow, Joint::BaseSmart *shoulder, double elbowLength, double shoulderLength) {
	Elbow = elbow;
	Shoulder = shoulder;
	ElbowLength = elbowLength;
	ShoulderLength = shoulderLength;
}

Arm::~Arm() {
	// empty
}

double Arm::GetX() {
	double elbow_x = ElbowLength * Tools::DegCos(Elbow->GetAngle());
	double shoulder_x = ShoulderLength * Tools::DegCos(Shoulder->GetAngle());
	return elbow_x + shoulder_x;
}

double Arm::GetY() {
	double elbow_y = ElbowLength * Tools::DegSin(Elbow->GetAngle());
	double shoulder_y = ShoulderLength * Tools::DegSin(Shoulder->GetAngle());
	return elbow_y + shoulder_y;
}

double Arm::GetAngle() {
	return Tools::DegAtan(GetY()/GetX());
}

double Arm::GetMagnitude() {
	return Tools::FindMagnitude(GetY(), GetX());    
}


SetJointSpeedCommand::SetJointSpeedCommand(Joint::Base *joint, double speed) :
		SimpleCommand("SetJointSpeed", true),
		m_speed(speed) {
	m_joint = joint;
	Requires(joint);
}

SetJointSpeedCommand::~SetJointSpeedCommand() {
	// empty
}

void SetJointSpeedCommand::Execute() {
	m_joint->SetSpeed(m_speed);
}



SetPolarCommand::SetPolarCommand(Arm arm, double angle, double magnitude) :
		Command("SetPolar"),
		m_arm(arm),
		m_angle(angle),
		m_magnitude(magnitude),
		m_angleRange(3.0),
		m_magnitudeRange(2.0) {
}

SetPolarCommand::~SetPolarCommand() {
	//
}

void SetPolarCommand::Initialize() {
	//
}

void SetPolarCommand::Execute() {
    double shoulder_angle = m_angle + Tools::FindAngleOnTriangle(
    		m_arm.ElbowLength, 
    		m_magnitude, 
    		m_arm.ShoulderLength);
    double elbow_angle = 180 - Tools::FindAngleOnTriangle(
    		m_arm.ElbowLength, 
    		m_magnitude, 
    		m_arm.ShoulderLength);
    
    m_arm.Shoulder->SetAngle(shoulder_angle);
    m_arm.Elbow->SetAngle(elbow_angle);
}

bool SetPolarCommand::IsFinished() {
	bool isAngleWithinRange = Tools::IsWithinRange(m_arm.GetAngle(), m_angle, m_angleRange);
	bool isMagnitudeWithinRange = Tools::IsWithinRange(m_arm.GetMagnitude(), m_magnitude, m_magnitudeRange); 
	return isAngleWithinRange && isMagnitudeWithinRange; 
}

void SetPolarCommand::End() {
	// empty
}

void SetPolarCommand::Interrupted() {
	// empty
}


SetCartesianCommand::SetCartesianCommand(Arm arm, double x, double y) :
		CommandGroup("SetCartesian") {
	double magnitude = Tools::FindMagnitude(x, y);
	double angle = Tools::DegAtan(y/x);
	
	AddSequential(new SetPolarCommand(arm, angle, magnitude));
}

SetCartesianCommand::~SetCartesianCommand() {
	//
}


SetDegreesCommand::SetDegreesCommand(Arm arm, double degrees) :
		CommandGroup("SetDegrees") {
	AddSequential(new SetPolarCommand(arm, degrees, arm.GetMagnitude()));
}

SetDegreesCommand::~SetDegreesCommand() {
	//
}


SetMagnitudeCommand::SetMagnitudeCommand(Arm arm, double magnitude) :
		CommandGroup("SetMagnitude") {
	AddSequential(new SetPolarCommand(arm, arm.GetAngle(), magnitude));
}

SetMagnitudeCommand::~SetMagnitudeCommand() {
	//
}


SetXCommand::SetXCommand(Arm arm, double x) :
		CommandGroup("SetX") {
	AddSequential(new SetCartesianCommand(arm, x, arm.GetY()));
}

SetXCommand::~SetXCommand() {
	//
}


SetYCommand::SetYCommand(Arm arm, double y) :
		CommandGroup("SetY") {
	AddSequential(new SetCartesianCommand(arm, arm.GetX(), y));
}

SetYCommand::~SetYCommand() {
	//
}

ControlWinchManualCommand::ControlWinchManualCommand(Winch::Base *winch, Axis *axis) :
		SimpleCommand("ControlWinch", false) {
	m_winch = winch;
	m_axis = axis;
}

ControlWinchManualCommand::~ControlWinchManualCommand() {
	//
}

void ControlWinchManualCommand::Execute() {
	m_winch->SetSpeed(m_axis->Get());
}
