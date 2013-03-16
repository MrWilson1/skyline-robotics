#include "BasicArmCommands.h"

BasicArmCommand::Arm::Arm(Joint::BaseSmart *elbow, Joint::BaseSmart *shoulder, double elbowLength, double shoulderLength) {
	Elbow = elbow;
	Shoulder = shoulder;
	ElbowLength = elbowLength;
	ShoulderLength = shoulderLength;
}

BasicArmCommand::Arm::~Arm() {
	// empty
}

double BasicArmCommand::Arm::GetX() {
	double elbow_x = ElbowLength * Tools::DegCos(Elbow->GetAngle());
	double shoulder_x = ShoulderLength * Tools::DegCos(Shoulder->GetAngle());
	return elbow_x + shoulder_x;
}

double BasicArmCommand::Arm::GetY() {
	double elbow_y = ElbowLength * Tools::DegSin(Elbow->GetAngle());
	double shoulder_y = ShoulderLength * Tools::DegSin(Shoulder->GetAngle());
	return elbow_y + shoulder_y;
}

double BasicArmCommand::Arm::GetAngle() {
	return Tools::DegAtan(GetY()/GetX());
}

double BasicArmCommand::Arm::GetMagnitude() {
	return Tools::FindMagnitude(GetY(), GetX());    
}



BasicArmCommand::SetJointSpeed::SetJointSpeed(Joint::Base *joint, double speed) :
		SimpleCommand("SetJointSpeed", true),
		m_speed(speed) {
	m_joint = joint;
	Requires(joint);
}

BasicArmCommand::SetJointSpeed::~SetJointSpeed() {
	// empty
}

void BasicArmCommand::SetJointSpeed::Execute() {
	m_joint->SetSpeed(m_speed);
}



BasicArmCommand::SetPolar::SetPolar(Arm arm, double angle, double magnitude) :
		Command("SetPolar"),
		m_arm(arm),
		m_angle(angle),
		m_magnitude(magnitude),
		m_angleRange(3.0),
		m_magnitudeRange(2.0) {
}

BasicArmCommand::SetPolar::~SetPolar() {
	//
}

void BasicArmCommand::SetPolar::Initialize() {
	//
}

void BasicArmCommand::SetPolar::Execute() {
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

bool BasicArmCommand::SetPolar::IsFinished() {
	bool isAngleWithinRange = Tools::IsWithinRange(m_arm.GetAngle(), m_angle, m_angleRange);
	bool isMagnitudeWithinRange = Tools::IsWithinRange(m_arm.GetMagnitude(), m_magnitude, m_magnitudeRange); 
	return isAngleWithinRange && isMagnitudeWithinRange; 
}

void BasicArmCommand::SetPolar::End() {
	// empty
}

void BasicArmCommand::SetPolar::Interrupted() {
	// empty
}


BasicArmCommand::SetCartesian::SetCartesian(Arm arm, double x, double y) :
		CommandGroup("SetCartesian") {
	double magnitude = Tools::FindMagnitude(x, y);
	double angle = Tools::DegAtan(y/x);
	
	AddSequential(new SetPolar(arm, angle, magnitude));
}

BasicArmCommand::SetCartesian::~SetCartesian() {
	//
}


BasicArmCommand::SetDegrees::SetDegrees(Arm arm, double degrees) :
		CommandGroup("SetDegrees") {
	AddSequential(new SetPolar(arm, degrees, arm.GetMagnitude()));
}

BasicArmCommand::SetDegrees::~SetDegrees() {
	//
}


BasicArmCommand::SetMagnitude::SetMagnitude(Arm arm, double magnitude) :
		CommandGroup("SetMagnitude") {
	AddSequential(new SetPolar(arm, arm.GetAngle(), magnitude));
}

BasicArmCommand::SetMagnitude::~SetMagnitude() {
	//
}


BasicArmCommand::SetX::SetX(Arm arm, double x) :
		CommandGroup("SetX") {
	AddSequential(new SetCartesian(arm, x, arm.GetY()));
}

BasicArmCommand::SetX::~SetX() {
	//
}


BasicArmCommand::SetY::SetY(Arm arm, double y) :
		CommandGroup("SetY") {
	AddSequential(new SetCartesian(arm, arm.GetX(), y));
}

BasicArmCommand::SetY::~SetY() {
	//
}

