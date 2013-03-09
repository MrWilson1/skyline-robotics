#include "ClimberExtender.h"

Extender::Base::Base(const char *name) :
	    BaseSubsystem(name) {
	// empty
}

Extender::Base::~Base() {
	// empty
}

Extender::Pneumatic::Pneumatic(Solenoid *rightSolenoid, Solenoid *leftSolenoid) :
	    Extender::Base("PneumaticClimberExtender") {
	m_rightSolenoid = rightSolenoid;
	m_leftSolenoid = leftSolenoid;
	AddActuatorToLiveWindow("Right Solenoid", m_rightSolenoid);
	AddActuatorToLiveWindow("Left Solenoid", m_leftSolenoid);
}

Extender::Pneumatic::~Pneumatic() {
	// empty
}

void Extender::Pneumatic::PullPins() {
	/* Pins will be manually reset at the beginning of each match. Here,
	 * setting the solenoids to the opposite state will pull the pins,
	 * and we won't have to worry about testing whether "True" or "False"
	 * corresponds with "pulled".
	 */
	m_rightSolenoid->Set(!m_rightSolenoid->Get());
	m_leftSolenoid->Set(!m_leftSolenoid->Get());
}
