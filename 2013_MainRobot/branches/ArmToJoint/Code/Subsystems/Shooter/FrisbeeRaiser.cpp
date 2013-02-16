#include "FrisbeeRaiser.h"

BaseFrisbeeRaiser::BaseFrisbeeRaiser(const char *name) :
		BaseSubsystem(name) {
	// Empty
}

BaseFrisbeeRaiser::~BaseFrisbeeRaiser() {
	// Empty
}

GuardedFrisbeeRaiser::GuardedFrisbeeRaiser(SpeedController *raiserMotor, DigitalInput *topLimitSwitch, DigitalInput *bottomLimitSwitch, double motorSpeed) :
		BaseFrisbeeRaiser("GuardedFrisbeeRaiser"),
		m_motorSpeed(motorSpeed) {
	m_raiserMotor = raiserMotor;
	m_topLimitSwitch = topLimitSwitch;
	m_bottomLimitSwitch = bottomLimitSwitch;
}

GuardedFrisbeeRaiser::~GuardedFrisbeeRaiser() {
	// empty
}

bool GuardedFrisbeeRaiser::IsAtTop() {
	// need to test limit switches to check which state returns true and which returns false
	return m_topLimitSwitch->Get();
}

bool GuardedFrisbeeRaiser::IsAtBottom() {
	// need to test limit switches to check which state returns true and which returns false
	return m_bottomLimitSwitch->Get();
}

void GuardedFrisbeeRaiser::Raise() {
	if (!GuardedFrisbeeRaiser::IsAtTop()) {
		m_raiserMotor->Set(m_motorSpeed);
	} else {
		Stop();
	}
}

void GuardedFrisbeeRaiser::Lower() {
	if (!GuardedFrisbeeRaiser::IsAtBottom()) {
		m_raiserMotor->Set(-m_motorSpeed);
	} else {
		Stop();
	}
}

void GuardedFrisbeeRaiser::Raise(float magnitude) {
	if (!GuardedFrisbeeRaiser::IsAtTop()) {
		m_raiserMotor->Set(Tools::Limit(magnitude, 0.0, 1.0));
	} else {
		Stop();
	}
}

void GuardedFrisbeeRaiser::Lower(float magnitude) {
	if (!GuardedFrisbeeRaiser::IsAtTop()) {
		m_raiserMotor->Set(-Tools::Limit(magnitude, 0.0, 1.0));
	} else {
		Stop();
	}
}

void GuardedFrisbeeRaiser::Stop() {
	m_raiserMotor->Set(0.0);
}
