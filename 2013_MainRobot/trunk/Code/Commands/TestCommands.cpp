#include "TestCommands.h"

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
