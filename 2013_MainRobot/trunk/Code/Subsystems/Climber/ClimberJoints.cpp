#include "ClimberJoints.h"

Joint::Base::Base(const char *name) :
		BaseSubsystem(name){
	//
}

Joint::Base::~Base() {
	//
}

Joint::BaseSmart::BaseSmart(const char *name) :
		Base(name) {
	//
}

Joint::BaseSmart::~BaseSmart() {
	//
}

Joint::BasicWinch::BasicWinch(SpeedController *motor) :
		Base("BaseWinchClimberJoint") {
	m_motor = motor;
	AddActuatorToLiveWindow("Joint Motor", m_motor);
}

Joint::BasicWinch::~BasicWinch() {
	//
}

void Joint::BasicWinch::SetSpeed(double speed) {
	m_motor->Set(Tools::Limit(speed, -1.0, 1.0));
}

double Joint::BasicWinch::GetSpeed() {
	return m_motor->Get();
}

Joint::PidWinch::PidWinch(SpeedController *motor, Encoder *encoder) :
		Joint::BaseSmart("MotorClimberJoint") {
	m_motor = motor;
	m_encoder = encoder;
	m_encoder->SetPIDSourceParameter(Encoder::kDistance);
	m_encoder->Start();
	m_pid = new PIDController(0.1, 0, 0, m_encoder, m_motor);
	m_pid->SetInputRange(-180, 180);
	m_pid->SetOutputRange(-180, 180);
	AddActuatorToLiveWindow("PID Controller", m_pid);
}

Joint::PidWinch::~PidWinch() {
	delete m_pid;
}

void Joint::PidWinch::SetAngle(float degrees) {
	m_pid->SetSetpoint(degrees);
}

double Joint::PidWinch::GetAngle() {
	return m_pid->Get();
}

void Joint::PidWinch::SetSpeed(double speed) {
	m_motor->Set(Tools::Limit(speed, -1.0, 1.0));
}

double Joint::PidWinch::GetSpeed() {
	return m_motor->Get();
}
