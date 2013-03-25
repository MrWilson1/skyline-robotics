#ifndef WINCH_COMMANDS_H
#define WINCH_COMMANDS_H

#include "WPILib.h"
#include "BaseCommand.h"
#include "../Misc/Tools.h"
#include "../Subsystems/Controllers/Axis.h"
#include "../Subsystems/Climber/ClimberWinch.h"

namespace WinchCommand {

/**
 * Command to control the winch using an axis.
 */
class ControlManual :  public SimpleCommand {
public:
	ControlManual(Winch::Base *winch, BaseAxis *axis);
	~ControlManual();
	
	void Execute();
private:
	Winch::Base *m_winch;
	BaseAxis *m_axis;
};


class SetSpeed : public Command {
public:
	SetSpeed(Winch::Base *winch, float speed);
	~SetSpeed();
	
	void Initialize();
	void Execute();
	bool IsFinished();
	void Interrupted();
	void End();
	
private:
	Winch::Base *m_winch;
	const float m_speed;
};

}

#endif
