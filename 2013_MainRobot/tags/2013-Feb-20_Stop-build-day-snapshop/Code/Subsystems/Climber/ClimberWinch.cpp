#include "ClimberWinch.h"

BaseWinch::BaseWinch(const char *name) :
		BaseSubsystem(name) {
	//
}

BaseWinch::~BaseWinch() {
	//
}

SimpleWinch::SimpleWinch(SpeedController *motor) :
		BaseWinch("SimpleWinch") {
	m_motor = motor;
}

SimpleWinch::~SimpleWinch() {
	//
}

void SimpleWinch::SetSpeed(double speed) {
	m_motor->Set(Tools::Limit(speed, -1.0, 1.0));
}

double SimpleWinch::GetSpeed() {
	return m_motor->Get();
}

