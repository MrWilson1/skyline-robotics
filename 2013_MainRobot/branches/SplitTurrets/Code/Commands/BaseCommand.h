#ifndef BASE_COMMAND_H
#define BASE_COMMAND_H
/**
 * \file BaseCommands.h
 * 
 * \addtogroup commands
 * \{
 */

#include "WPILib.h"

/*
 * Simple version of Command class that only requires child classes to define Execute() function.
 */
class SimpleCommand : public Command {
public:
	SimpleCommand(const char *name, bool valueToReturnForIsFinished);
	virtual ~SimpleCommand();

	virtual void Initialize();
	virtual void Execute() = 0;
	virtual bool IsFinished();
	virtual void End();
	virtual void Interrupted();
	
protected:
	bool m_valueToReturnForIsFinished;
};

/**
 * \}
 */
#endif
