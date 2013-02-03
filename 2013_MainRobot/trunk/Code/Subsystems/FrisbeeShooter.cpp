#include "FrisbeeShooter.h"

BaseFrisbeeShooter::BaseFrisbeeShooter(const char *name) :
		BaseSubsystem(name) {
	// Empty
}

BaseFrisbeeShooter::~BaseFrisbeeShooter() {
	// Empty
}




SimpleBeltShooter::SimpleBeltShooter(SpeedController *motor, Direction forwardDirection) :
		BaseFrisbeeShooter("SimpleBeltShooter"),
		m_forwardDirection(forwardDirection),
		m_maxDistance(120) {
	m_motor = motor;
	
	AddActuatorToLiveWindow("Belt motor", m_motor);
}

SimpleBeltShooter::~SimpleBeltShooter() {
	// Empty
}

void SimpleBeltShooter::ShootFrisbee() {
	m_motor->Set(1.0 * m_forwardDirection);
}

void SimpleBeltShooter::ShootFrisbee(float distanceInInches) {
	float reachableDistance = Tools::Limit(distanceInInches, 0, m_maxDistance);
	float motorPercentage = Tools::Scale(reachableDistance, 0, m_maxDistance, 0, 1);
	m_motor->Set(motorPercentage * m_forwardDirection);
}

void SimpleBeltShooter::EjectFrisbee() {
	m_motor->Set(0.2);
}

void SimpleBeltShooter::SetMaxDistance(float inches) {
	m_maxDistance = inches;
}
	
float SimpleBeltShooter::GetMaxDistance() {
	return m_maxDistance;
}



ThreeWheelShooter::ThreeWheelShooter(SpeedController *frontMotor, SpeedController *middleMotor, SpeedController *lastMotor) :
		BaseFrisbeeShooter("ThreeWheelShooter"),
		m_maxDistance(120) {
	m_frontMotor = frontMotor;
	m_middleMotor = middleMotor;
	m_lastMotor = lastMotor;
	AddActuatorToLiveWindow("Front Motor", m_frontMotor);
	AddActuatorToLiveWindow("Middle Motor", m_middleMotor);
	AddActuatorToLiveWindow("Last Motor", m_lastMotor);
}

ThreeWheelShooter::~ThreeWheelShooter() {
	// empty
}

void ThreeWheelShooter::ShootFrisbee() {
	m_frontMotor->Set(1.0);
	m_middleMotor->Set(1.0);
	m_lastMotor->Set(1.0);
}

void ThreeWheelShooter::ShootFrisbee(float distanceInInches) {
	// This needs calibration. For now, just do the default.
	ShootFrisbee();
}

void ThreeWheelShooter::EjectFrisbee() {
	m_frontMotor->Set(-0.1);
	m_middleMotor->Set(-0.1);
	m_lastMotor->Set(-0.1);
}

void ThreeWheelShooter::SetMaxDistance(float inches) {
	m_maxDistance = inches;
}

float ThreeWheelShooter::GetMaxdistance() {
	return m_maxDistance;
}
