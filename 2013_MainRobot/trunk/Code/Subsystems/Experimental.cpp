#include "Experimental.h"

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
