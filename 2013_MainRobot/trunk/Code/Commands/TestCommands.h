#ifndef TEST_COMMANDS_H
#define TEST_COMMANDS_H

#include "BaseCommand.h"
#include "../Subsystems/Experimental.h"
#include "../Subsystems/Controllers/Axis.h"

/**
 * \brief An experimental 
 */
class TestMotorCommand : public Command {
public:
	TestMotorCommand(TestMotor *testMotor, Axis *axis);
	~TestMotorCommand();

	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();

private:
	TestMotor *m_testMotor;
	Axis *m_axis;
};


class TestEncoderCommand : public SimpleCommand {
public:
	TestEncoderCommand(TestEncoder *testEncoder, const char *name);
	~TestEncoderCommand();
	void Execute();
private:
	TestEncoder *m_testEncoder;
};

class StartTestEncoderCommand : public SimpleCommand {
public:
	StartTestEncoderCommand(TestEncoder *testEncoder, const char *name);
	~StartTestEncoderCommand();
	void Execute();
private:
	TestEncoder *m_testEncoder;
};

class StopTestEncoderCommand : public SimpleCommand {
public:
	StopTestEncoderCommand(TestEncoder *testEncoder, const char *name);
	~StopTestEncoderCommand();
	void Execute();
private:
	TestEncoder *m_testEncoder;
};

class ResetTestEncoderCommand : public SimpleCommand {
public:
	ResetTestEncoderCommand(TestEncoder *testEncoder, const char *name);
	~ResetTestEncoderCommand();
	void Execute();
private:
	TestEncoder *m_testEncoder;
};

class UpdateTestEncoderCommand : public SimpleCommand {
public:
	UpdateTestEncoderCommand(TestEncoder *testEncoder, const char *name);
	~UpdateTestEncoderCommand();
	void Execute();
private:
	TestEncoder *m_testEncoder;
};

#endif
