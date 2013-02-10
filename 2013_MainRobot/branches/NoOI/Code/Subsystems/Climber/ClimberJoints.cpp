#include "ClimberJoints.h"

BaseClimberJoint::BaseClimberJoint(const char *name) :
		BaseSubsystem(name){
	//
}

BaseClimberJoint::~BaseClimberJoint() {
	//
}

MotorClimberJoint::MotorClimberJoint(SpeedController *motor, Encoder *encoder) :
		BaseClimberJoint("MotorClimberJoint") {
	m_motor = motor;
	m_encoder = encoder;
}

MotorClimberJoint::~MotorClimberJoint() {
	//
}

void MotorClimberJoint::SetAngle(float degrees) {
	//
}

float MotorClimberJoint::GetAngle() {
	//
}

WinchClimberJoint::WinchClimberJoint(SpeedController *motor, Encoder *encoder) :
		BaseClimberJoint("WinchClimberJoint"){
	m_motor = motor;
	m_encoder = encoder;
}

WinchClimberJoint::~WinchClimberJoint() {
	//
}

void WinchClimberJoint::SetAngle(float degrees) {
	//
}

float WinchClimberJoint::GetAngle() {
	//
}
