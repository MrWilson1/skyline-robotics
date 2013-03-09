#include "ClimberWinch.h"

Winch::Base::Base(const char *name) :
		BaseSubsystem(name) {
	//
}

Winch::Base::~Base() {
	//
}

Winch::Simple::Simple(SpeedController *motor) :
		Base("SimpleWinch") {
	m_motor = motor;
}

Winch::Simple::~Simple() {
	//
}

void Winch::Simple::SetSpeed(double speed) {
	m_motor->Set(Tools::Limit(speed, -1.0, 1.0));
}

double Winch::Simple::GetSpeed() {
	return m_motor->Get();
}

