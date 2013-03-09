#include "FrisbeeTurret.h"



BaseAxisFrisbeeTurret::BaseAxisFrisbeeTurret(const char *name) :
		BaseSubsystem(name) {
	// Empty
}
BaseAxisFrisbeeTurret::~BaseAxisFrisbeeTurret() {
	// Empty
}



SimpleAxisFrisbeeTurret::SimpleAxisFrisbeeTurret(SpeedController *motor) :
		BaseAxisFrisbeeTurret("BaseAxisFrisbeeTurret"),
		k_highSpeed(0.8),
		k_mediumSpeed(0.5),
		k_lowSpeed(0.3) {
	m_motor = motor; 
	AddActuatorToLiveWindow("Motor", m_motor);
}

SimpleAxisFrisbeeTurret::~SimpleAxisFrisbeeTurret() {
	//empty
}

void SimpleAxisFrisbeeTurret::SetMotor(float speed){
	m_motor->Set(speed); 
}

void SimpleAxisFrisbeeTurret::Stop(){
	m_motor->Set(0);
}

bool SimpleAxisFrisbeeTurret::ShouldTurretStop(){
	return false; 
}

void SimpleAxisFrisbeeTurret::TurnGivenOffset(Tracking::Offset offset, double direction, double upperBound, double middleBound, double lowerBound) {
	if (Tools::IsWithinBounds(upperBound, middleBound, fabs(offset.XOffset))) {
		SetMotor(direction * k_highSpeed);
	} else if (Tools::IsWithinBounds(middleBound, lowerBound, fabs(offset.XOffset))) {
		SetMotor(direction * k_mediumSpeed);
	} else if (Tools::IsWithinBounds(lowerBound, 0, fabs(offset.XOffset))) {
		SetMotor(direction * k_lowSpeed);
	}
}


GuardedAxisFrisbeeTurret::GuardedAxisFrisbeeTurret( SpeedController *motor, 
		 DigitalInput *switch1,
		 DigitalInput *switch2) :
		BaseAxisFrisbeeTurret("GuardedAxisFrsibeeTurret"),
		k_highSpeed(0.8),
		k_mediumSpeed(0.5),
		k_lowSpeed(0.3) {
	m_motor = motor;
	m_switch1 = switch1;
	m_switch2 = switch2;
	AddActuatorToLiveWindow("Motor", m_motor);
	AddSensorToLiveWindow("Switch 1", m_switch1);
	AddSensorToLiveWindow("Switch 2", m_switch2);
}

GuardedAxisFrisbeeTurret::~GuardedAxisFrisbeeTurret() {
	//empty
}
void GuardedAxisFrisbeeTurret::SetMotor(float speed){
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

void GuardedAxisFrisbeeTurret::Stop(){
	m_motor->Set(0);
}

bool GuardedAxisFrisbeeTurret::ShouldTurretStop(){
	//return !m_switch1->Get() || !m_switch2->Get();
	// todo: purge
	return false;
}

void GuardedAxisFrisbeeTurret::TurnGivenOffset(Tracking::Offset offset, double direction, double upper, double middle, double lower) {
	if (Tools::IsWithinBounds(upper, middle, fabs(offset.XOffset))) {
		SetMotor(direction * k_highSpeed);
	} else if (Tools::IsWithinBounds(middle, lower, fabs(offset.XOffset))) {
		SetMotor(direction * k_mediumSpeed);
	} else if (Tools::IsWithinBounds(lower, 0, fabs(offset.XOffset))) {
		SetMotor(direction * k_lowSpeed);
	}
}

