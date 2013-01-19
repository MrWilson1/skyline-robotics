#ifndef TEST_COMMANDS_H
#define TEST_COMMANDS_H

#include "../Subsystems/Experimental.h"

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
