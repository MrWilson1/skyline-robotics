#include "BasicArmCommands.h"

ArmData::ArmData(BaseSmartJoint *elbow, BaseSmartJoint *shoulder, double elbowLength, double shoulderLength) {
	m_elbow = elbow;
	m_shoulder = shoulder;
	m_elbowLength = elbowLength;
	m_shoulderLength = shoulderLength;
}

double Arm::GetX(ArmData arm) {
	double elbow_x = arm.m_elbowLength * Tools::DegCos(arm.m_elbow->GetAngle());
	double shoulder_x = arm.m_shoulderLength * Tools::DegCos(arm.m_shoulder->GetAngle());
	return elbow_x + shoulder_x;
}

double Arm::GetY(ArmData arm) {
	double elbow_y = arm.m_elbowLength * Tools::DegSin(arm.m_elbow->GetAngle());
	double shoulder_y = arm.m_shoulderLength * Tools::DegSin(arm.m_shoulder->GetAngle());
	return elbow_y + shoulder_y;
}

double Arm::GetAngle(ArmData arm) {
	return Tools::DegAtan(GetY(arm)/GetX(arm));
}

double Arm::GetMagnitude(ArmData arm) {
	return Tools::FindMagnitude(GetY(arm), GetX(arm));    
}

SetJointSpeedCommand::SetJointSpeedCommand(BaseJoint *joint, double speed) :
		Command("SetJointSpeed"),
		m_speed(speed) {
	m_joint = joint;
	Requires(joint);
}

SetJointSpeedCommand::~SetJointSpeedCommand() {
	//
}

void SetJointSpeedCommand::Initialize() {
	//
}

void SetJointSpeedCommand::Execute() {
	m_joint->SetSpeed(m_speed);
}

bool SetJointSpeedCommand::IsFinished() {
	return true;
}

void SetJointSpeedCommand::Interrupted() {
	//
}

void SetJointSpeedCommand::End() {
	//
}

SetPolarCommand::SetPolarCommand(ArmData arm, double angle, double magnitude) :
		Command("SetPolar"),
		m_armData(arm),
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
    double shoulder_angle = m_angle + Tools::FindAngleOnTriangle(m_armData.m_elbowLength, m_magnitude, m_armData.m_shoulderLength);
    double elbow_angle = 180 - Tools::FindAngleOnTriangle(m_armData.m_elbowLength, m_magnitude, m_armData.m_shoulderLength);
    
    m_armData.m_shoulder->SetAngle(shoulder_angle);
    m_armData.m_elbow->SetAngle(elbow_angle);
}

bool SetPolarCommand::IsFinished() {
	return Tools::IsWithinRange(Arm::GetAngle(m_armData), m_angle, m_angleRange) && Tools::IsWithinRange(Arm::GetMagnitude(m_armData), m_magnitude, m_magnitudeRange);
}

void SetPolarCommand::Interrupted() {
	//
}

void SetPolarCommand::End() {
	//
}

SetCartesianCommand::SetCartesianCommand(ArmData arm, double x, double y) :
		CommandGroup("SetCartesian") {
	double magnitude = Tools::FindMagnitude(x, y);
	double angle = Tools::DegAtan(y/x);
	
	AddSequential(new SetPolarCommand(arm, angle, magnitude));
}

SetCartesianCommand::~SetCartesianCommand() {
	//
}

SetDegreesCommand::SetDegreesCommand(ArmData arm, double degrees) :
		CommandGroup("SetDegrees") {
	AddSequential(new SetPolarCommand(arm, degrees, Arm::GetMagnitude(arm)));
}

SetDegreesCommand::~SetDegreesCommand() {
	//
}

SetMagnitudeCommand::SetMagnitudeCommand(ArmData arm, double magnitude) :
		CommandGroup("SetMagnitude") {
	AddSequential(new SetPolarCommand(arm, Arm::GetAngle(arm), magnitude));
}

SetMagnitudeCommand::~SetMagnitudeCommand() {
	//
}

SetXCommand::SetXCommand(ArmData arm, double x) :
		CommandGroup("SetX") {
	AddSequential(new SetCartesianCommand(arm, x, Arm::GetY(arm)));
}

SetXCommand::~SetXCommand() {
	//
}

SetYCommand::SetYCommand(ArmData arm, double y) :
		CommandGroup("SetY") {
	AddSequential(new SetCartesianCommand(arm, Arm::GetX(arm), y));
}

SetYCommand::~SetYCommand() {
	//
}
