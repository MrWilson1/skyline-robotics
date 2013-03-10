#ifndef BASE_COMMAND_H
#define BASE_COMMAND_H
/**
 * \file BaseCommand.h
 * 
 * \defgroup commands Commands
 * 
 * \brief Corresponds to robot behavior.
 * 
 * Commands combine subsystems and other commands to script out 
 * complex behavior. 
 * 
 * Note: each Command class needs to call the `Require` method on
 * any passed-in subsystems it uses. If it doesn't, then the command 
 * may not run.
 * 
 * Also note that in order to specify a default command for a given
 * subsystem, you need to call the `SetDefaultCommand` method on
 * the subsystem.
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
