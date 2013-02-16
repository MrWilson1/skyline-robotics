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
	m_encoder->SetPIDSourceParameter(Encoder::kDistance);
	m_encoder->Start();
	m_pid = new PIDController(0.1, 0, 0, m_encoder, m_motor);
	m_pid->SetInputRange(-180, 180);
	m_pid->SetOutputRange(-180, 180);
	AddActuatorToLiveWindow("PID Controller", m_pid);
}

MotorClimberJoint::~MotorClimberJoint() {
	//
}

void MotorClimberJoint::SetAngle(float degrees) {
	m_pid->SetSetpoint(degrees);
}

float MotorClimberJoint::GetAngle() {
	return m_pid->Get();
}

WinchClimberJoint::WinchClimberJoint(SpeedController *motor, Encoder *encoder) :
		BaseClimberJoint("WinchClimberJoint"){
	m_motor = motor;
	m_encoder = encoder;
	m_encoder->SetPIDSourceParameter(Encoder::kDistance);
	m_encoder->Start();
	m_pid = new PIDController(0.1, 0, 0, m_encoder, m_motor);
	m_pid->SetInputRange(-180, 180);
	m_pid->SetOutputRange(-180, 180);
	AddActuatorToLiveWindow("PID Controller", m_pid);
}

WinchClimberJoint::~WinchClimberJoint() {
	//
}

void WinchClimberJoint::SetAngle(float degrees) {
	m_pid->SetSetpoint(degrees);
}

float WinchClimberJoint::GetAngle() {
	return m_pid->Get();
}
