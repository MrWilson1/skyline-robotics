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
		BaseSubsystem(name),
		m_distancePerPulse(1) {
	m_encoder = encoder;
	//m_encoder->SetDistancePerPulse(m_distancePerPulse);
	AddSensorToLiveWindow("Raw Encoder", m_encoder);
}

TestEncoder::TestEncoder(Encoder *encoder, float distancePerPulse, const char *name) :
		BaseSubsystem(name),
		m_distancePerPulse(distancePerPulse) {
	m_encoder = encoder;
	//m_encoder->SetDistancePerPulse(m_distancePerPulse);
	AddSensorToLiveWindow("Raw Encoder", m_encoder);
}

TestEncoder::~TestEncoder() {
	// Empty
}

void TestEncoder::ReportAll() {
	Log("Get", m_encoder->Get());
	Log("Get raw", m_encoder->GetRaw());
	Log("Period", m_encoder->GetPeriod());
	Log("Rate", m_encoder->GetRate());
	LogBool("Is encoder stopped?", m_encoder->GetStopped());
	Log("Distance", m_encoder->GetDistance());
	Log("PIDGet", m_encoder->PIDGet());
}

void TestEncoder::Start() {
	m_encoder->Start();
}

void TestEncoder::Stop() {
	m_encoder->Stop();
}

void TestEncoder::Reset() {
	m_encoder->Reset();
}

void TestEncoder::SetMaxPeriod(double maxPeriod) {
	m_encoder->SetMaxPeriod(maxPeriod);
}

void TestEncoder::SetMinRate(double minRate) {
	m_encoder->SetMinRate(minRate);
}

void TestEncoder::SetDistancePerPulse(double input) {
	m_distancePerPulse = input;
	m_encoder->SetDistancePerPulse(input);
}

void TestEncoder::SetReverseDirection(bool reverseDirection) {
	m_encoder->SetReverseDirection(reverseDirection);
}

void TestEncoder::SetPIDSourceParameter(Encoder::PIDSourceParameter pidSource) {
	m_encoder->SetPIDSourceParameter(pidSource);
}

double TestEncoder::GetDistancePerPulse() {
	return m_distancePerPulse;
}

void TestEncoder::Log(const char *key, int value) {
	SmartDashboard::PutNumber(GetName() + std::string(" ") + std::string(key), value);
}

void TestEncoder::Log(const char *key, double value) {
	SmartDashboard::PutNumber(GetName() + std::string(" ") + std::string(key), value);
}

void TestEncoder::LogBool(const char *key, bool value) {
	SmartDashboard::PutBoolean(GetName() + std::string(" ") + std::string(key), value);
}



ReversedVictor::ReversedVictor(UINT32 channel) :
		Victor(channel) {
	// empty
}

ReversedVictor::ReversedVictor(UINT8 moduleNumber, UINT32 channel) :
		Victor(moduleNumber, channel) {
	// empty
}

ReversedVictor::~ReversedVictor() {
	// empty
}
	
void ReversedVictor::Set(float speed) {
	Victor::Set(-speed);
}

void ReversedVictor::Set(float speed, UINT8 syncGroup) {
	Victor::Set(-speed, syncGroup);
}

float ReversedVictor::Get() {
	return -Victor::Get();
}

void ReversedVictor::Disable() {
	Victor::Disable();
}

void ReversedVictor::PIDWrite(float output) {
	Victor::PIDWrite(-output);
}

