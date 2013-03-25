#include "TestCommands.h"

TestMotorCommand::TestMotorCommand(TestMotor *testMotor, BaseAxis *axis) :
		Command("TestMotorCommand1") {
	m_testMotor = testMotor;
	m_axis = axis;
	
	Requires(m_testMotor);
}

TestMotorCommand::~TestMotorCommand() {
	// empty
}

void TestMotorCommand::Initialize() {
	m_testMotor->SetSpeed(0);
}

void TestMotorCommand::Execute() {
	m_testMotor->SetSpeed(m_axis->Get());
}

bool TestMotorCommand::IsFinished() {
	return false;
}

void TestMotorCommand::End() {
	m_testMotor->SetSpeed(0);
}

void TestMotorCommand::Interrupted() {
	m_testMotor->SetSpeed(0);
}




TestEncoderCommand::TestEncoderCommand(TestEncoder *testEncoder, const char *name) :
		SimpleCommand(name, true) {
	m_testEncoder = testEncoder;
	Requires(m_testEncoder);
}

TestEncoderCommand::~TestEncoderCommand() {
	// empty
}

void TestEncoderCommand::Execute() {
	m_testEncoder->ReportAll();
}


StartTestEncoderCommand::StartTestEncoderCommand(TestEncoder *testEncoder, const char *name) :
		SimpleCommand(name, false) {
	m_testEncoder = testEncoder;
	Requires(m_testEncoder);
}

StartTestEncoderCommand::~StartTestEncoderCommand() {
	// empty
}
	
void StartTestEncoderCommand::Execute() {
	m_testEncoder->Start();
}


StopTestEncoderCommand::StopTestEncoderCommand(TestEncoder *testEncoder, const char *name) :
		SimpleCommand(name, false) {
	m_testEncoder = testEncoder;
	Requires(m_testEncoder);
}

StopTestEncoderCommand::~StopTestEncoderCommand() {
	// empty
}
	
void StopTestEncoderCommand::Execute() {
	m_testEncoder->Stop();
}


ResetTestEncoderCommand::ResetTestEncoderCommand(TestEncoder *testEncoder, const char *name) :
		SimpleCommand(name, false) {
	m_testEncoder = testEncoder;
	Requires(m_testEncoder);
}

ResetTestEncoderCommand::~ResetTestEncoderCommand() {
	// empty
}
	
void ResetTestEncoderCommand::Execute() {
	m_testEncoder->Reset();
}


UpdateTestEncoderCommand::UpdateTestEncoderCommand(TestEncoder *testEncoder, const char *name) :
		SimpleCommand(name, false) {
	m_testEncoder = testEncoder;
	Requires(m_testEncoder);
}

UpdateTestEncoderCommand::~UpdateTestEncoderCommand() {
	// empty
}

void UpdateTestEncoderCommand::Execute() {
	std::string key = m_testEncoder->GetName() + std::string(" ") + std::string("SetDistancePerPulse");
	double number = SmartDashboard::GetNumber(key);
	m_testEncoder->SetDistancePerPulse(number);
}
