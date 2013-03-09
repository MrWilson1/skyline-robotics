#include "FrisbeeTurret.h"

FrisbeeTurret::Base::Base(const char *name) :
		BaseSubsystem(name) {
	// Empty
}
FrisbeeTurret::Base::~Base() {
	// Empty
}



FrisbeeTurret::Simple::Simple(SpeedController *motor) :
		FrisbeeTurret::Base("FrisbeeTurret::Simple") {
	m_motor = motor; 
	AddActuatorToLiveWindow("Motor", m_motor);
}

FrisbeeTurret::Simple::~Simple() {
	//empty
}

void FrisbeeTurret::Simple::SetSpeed(float speed){
	m_motor->Set(speed); 
}

void FrisbeeTurret::Simple::Stop(){
	m_motor->Set(0);
}



FrisbeeTurret::Guarded::Guarded(
		SpeedController *motor, 
		DigitalInput *switch1,
		DigitalInput *switch2) :
		FrisbeeTurret::Base("FrisbeeTurret::Guarded") {
	m_motor = motor;
	m_switch1 = switch1;
	m_switch2 = switch2;
	AddActuatorToLiveWindow("Motor", m_motor);
	AddSensorToLiveWindow("Switch 1", m_switch1);
	AddSensorToLiveWindow("Switch 2", m_switch2);
}

FrisbeeTurret::Guarded::~Guarded() {
	//empty
}
void FrisbeeTurret::Guarded::SetSpeed(float speed){
	if (m_switch1->Get() && speed > 0.0) {
		m_motor->Set(0.0);
		return;
	}
	if (m_switch2->Get() && speed < 0.0) {
		m_motor->Set(0.0);
		return;
	}
		
	m_motor->Set(speed);
}

void FrisbeeTurret::Guarded::Stop(){
	m_motor->Set(0);
}


