#include "ClimberArm.h"

BaseClimberArm::BaseClimberArm(const char *name) :
		BaseSubsystem(name) {
	// empty
}

BaseClimberArm::~BaseClimberArm() {
	// empty
}

BungeeWinchClimberArm::BungeeWinchClimberArm(BaseClimberJoint *elbow, BaseClimberJoint *shoulder, double elbowLength, double shoulderLength) :
		BaseClimberArm("BungeeWinchClimberArm"),
		m_elbowLength(elbowLength),
		m_shoulderLength(shoulderLength) {
	m_elbow = elbow;
	m_shoulder = shoulder;
}

BungeeWinchClimberArm::~BungeeWinchClimberArm() {
	// empty
}

void BungeeWinchClimberArm::SetPolar(double degrees, double magnitude) {
	double shoulder_angle = degrees + Tools::FindAngleOnTriangle(m_elbowLength, magnitude, m_shoulderLength);
	double elbow_angle = 180 - Tools::FindAngleOnTriangle(m_shoulderLength, magnitude, m_elbowLength);
	
	m_shoulder->SetAngle(shoulder_angle);
	m_elbow->SetAngle(elbow_angle);
}

void BungeeWinchClimberArm::SetCartesian(double x, double y) {
	double magnitude = Tools::FindMagnitude(x, y);
	double angle = Tools::DegAtan(y/x);
	
	SetPolar(angle, magnitude);
}

void BungeeWinchClimberArm::SetDegrees(double degrees) {
	SetPolar(degrees, GetMagnitude());
}

void BungeeWinchClimberArm::SetMagnitude(double magnitude) {
	SetPolar(GetAngle(), magnitude);
}

void BungeeWinchClimberArm::SetX(double x) {
	SetCartesian(x, GetY());
}

void BungeeWinchClimberArm::SetY(double y) {
	SetCartesian(GetX(), y);
}

double BungeeWinchClimberArm::GetAngle() {
	return Tools::DegAtan(GetY()/GetX());
}

double BungeeWinchClimberArm::GetMagnitude() {
	return Tools::FindMagnitude(GetY(), GetX());	
}

double BungeeWinchClimberArm::GetX() {
	double elbow_x = m_elbowLength * Tools::DegCos(m_elbow->GetAngle());
	double shoulder_x = m_shoulderLength * Tools::DegCos(m_shoulder->GetAngle());
	return elbow_x + shoulder_x;
}

double BungeeWinchClimberArm::GetY() {
	double elbow_y = m_elbowLength * Tools::DegSin(m_elbow->GetAngle());
	double shoulder_y = m_shoulderLength * Tools::DegSin(m_shoulder->GetAngle());
	return elbow_y + shoulder_y;
}
