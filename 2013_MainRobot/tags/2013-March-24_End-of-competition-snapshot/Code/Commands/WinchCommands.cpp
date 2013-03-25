#include "WinchCommands.h"

WinchCommand::ControlManual::ControlManual(
		Winch::Base *winch, 
		BaseAxis *axis) :
		SimpleCommand("WinchCommand::ControlManual", false) {
	m_winch = winch;
	m_axis = axis;
	Requires(m_winch);
}

WinchCommand::ControlManual::~ControlManual() {
	//
}

void WinchCommand::ControlManual::Execute() {
	m_winch->SetSpeed(m_axis->Get());
}



WinchCommand::SetSpeed::SetSpeed(Winch::Base *winch, float speed) :
		Command("WinchCommand::SetSpeed"),
		m_speed(speed) {
	m_winch = winch;
	Requires(m_winch);
}

WinchCommand::SetSpeed::~SetSpeed() {
	// empty
}

void WinchCommand::SetSpeed::Initialize() {
	// empty
}

void WinchCommand::SetSpeed::Execute() {
	m_winch->SetSpeed(m_speed);
}

bool WinchCommand::SetSpeed::IsFinished() {
	return false;
}

void WinchCommand::SetSpeed::Interrupted() {
	m_winch->SetSpeed(0);
}

void WinchCommand::SetSpeed::End() {
	m_winch->SetSpeed(0);
}
