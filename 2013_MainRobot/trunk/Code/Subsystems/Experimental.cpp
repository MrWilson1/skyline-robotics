#include "Experimental.h"


TestMotor::TestMotor(SpeedController *motor, const char *name) :
		BaseSubsystem(name) {
	m_motor = motor;
	AddActuatorToLiveWindow("Motor", m_motor);
}

TestMotor::~TestMotor() {
	// Empty
}

void TestMotor::SetSpeed(float speed) {
	m_motor->Set(speed);
}

void TestMotor::MaxSpeed() {
	m_motor->Set(1.0);
}

TestEncoder::TestEncoder(Encoder *encoder, const char *name) :
		BaseSubsystem(name) {
	m_encoder = encoder;
	AddSensorToLiveWindow("Raw Encoder", m_encoder);
}

TestEncoder::~TestEncoder() {
	// Empty
}

void TestEncoder::SetDistancePerPulse(double input) {
	m_encoder->SetDistancePerPulse(input);
}

void TestEncoder::Run() {
	SmartDashboard::PutNumber(GetName() + std::string(" Rate"), m_encoder->GetRate());
}
