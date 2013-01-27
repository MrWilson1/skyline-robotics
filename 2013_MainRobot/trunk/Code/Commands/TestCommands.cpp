#include "TestCommands.h"

TestMotorCommand::TestMotorCommand(TestMotor *testMotor, float speed) :
		Command("TestMotorCommand1"),
		m_speed(speed) {
	m_testMotor = testMotor;
	
	Requires(m_testMotor);
}

TestMotorCommand::~TestMotorCommand() {
	// empty
}

void TestMotorCommand::Initialize() {
	m_testMotor->SetSpeed(0);
}

void TestMotorCommand::Execute() {
	m_testMotor->SetSpeed(m_speed);
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




TestEncoderCommand::TestEncoderCommand(TestEncoder *testEncoder) :
		Command("TestEncoderCommand") {
	m_testEncoder = testEncoder;
	
	Requires(m_testEncoder);
}

TestEncoderCommand::~TestEncoderCommand() {
	// empty
}

void TestEncoderCommand::Initialize() {
	// empty
}

void TestEncoderCommand::Execute() {
	m_testEncoder->Run();
}

bool TestEncoderCommand::IsFinished() {
	return false;
}

void TestEncoderCommand::End() {
	// empty
}

void TestEncoderCommand::Interrupted() {
	// empty
}
