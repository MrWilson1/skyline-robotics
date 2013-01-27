#ifndef TEST_COMMANDS_H
#define TEST_COMMANDS_H

#include "../Subsystems/Experimental.h"

/**
 * \brief An experimental 
 */
class TestMotorCommand : public Command {
public:
	TestMotorCommand(TestMotor *testMotor, float speed);
	~TestMotorCommand();

	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();

private:
	TestMotor *m_testMotor;
	float m_speed;
};


class TestEncoderCommand : public Command {
public:
	TestEncoderCommand(TestEncoder *testEncoder);
	~TestEncoderCommand();

	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();

private:
	TestEncoder *m_testEncoder;
};

#endif
