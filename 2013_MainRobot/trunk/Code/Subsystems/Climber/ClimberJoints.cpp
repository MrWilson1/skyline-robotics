#include "ClimberJoints.h"

BaseJoint::BaseJoint(const char *name) :
		BaseSubsystem(name){
	//
}

BaseJoint::~BaseJoint() {
	//
}

BaseSmartJoint::BaseSmartJoint(const char *name) :
		BaseJoint(name) {
	//
}

BaseSmartJoint::~BaseSmartJoint() {
	//
}

BasicWinchClimberJoint::BasicWinchClimberJoint(SpeedController *motor) :
		BaseJoint("BaseWinchClimberJoint") {
	m_motor = motor;
	AddActuatorToLiveWindow("Joint Motor", m_motor);
}

BasicWinchClimberJoint::~BasicWinchClimberJoint() {
	//
}

void BasicWinchClimberJoint::SetSpeed(double speed) {
	m_motor->Set(Tools::Limit(speed, -1.0, 1.0));
}

double BasicWinchClimberJoint::GetSpeed() {
	return m_motor->Get();
}

PidWinchClimberJoint::PidWinchClimberJoint(SpeedController *motor, Encoder *encoder) :
		BaseSmartJoint("MotorClimberJoint") {
	m_motor = motor;
	m_encoder = encoder;
	m_encoder->SetPIDSourceParameter(Encoder::kDistance);
	m_encoder->Start();
	m_pid = new PIDController(0.1, 0, 0, m_encoder, m_motor);
	m_pid->SetInputRange(-180, 180);
	m_pid->SetOutputRange(-180, 180);
	AddActuatorToLiveWindow("PID Controller", m_pid);
}

PidWinchClimberJoint::~PidWinchClimberJoint() {
	//
}

void PidWinchClimberJoint::SetAngle(float degrees) {
	m_pid->SetSetpoint(degrees);
}

double PidWinchClimberJoint::GetAngle() {
	return m_pid->Get();
}

void PidWinchClimberJoint::SetSpeed(double speed) {
	m_motor->Set(Tools::Limit(speed, -1.0, 1.0));
}

double PidWinchClimberJoint::GetSpeed() {
	return m_motor->Get();
}
