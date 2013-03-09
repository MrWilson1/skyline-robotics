#include "FrisbeeShooter.h"

FrisbeeShooter::Base::Base(const char *name) :
		BaseSubsystem(name) {
	// Empty
}

FrisbeeShooter::Base::~Base() {
	// Empty
}




FrisbeeShooter::SimpleBelt::SimpleBelt(SpeedController *motor, Direction forwardDirection) :
		FrisbeeShooter::Base("FrisbeeShooter::SimpleBelt"),
		m_forwardDirection(forwardDirection),
		m_maxDistance(120) {
	m_motor = motor;
	
	AddActuatorToLiveWindow("Belt motor", m_motor);
}

FrisbeeShooter::SimpleBelt::~SimpleBelt() {
	// Empty
}

void FrisbeeShooter::SimpleBelt::ShootFrisbee() {
	m_motor->Set(1.0 * m_forwardDirection);
}

void FrisbeeShooter::SimpleBelt::ShootFrisbee(float distanceInInches) {
	float reachableDistance = Tools::Limit(distanceInInches, 0, m_maxDistance);
	float motorPercentage = Tools::Scale(reachableDistance, 0, m_maxDistance, 0, 1);
	m_motor->Set(motorPercentage * m_forwardDirection);
}

void FrisbeeShooter::SimpleBelt::StopFrisbee() {
	m_motor->Set(0);
}

void FrisbeeShooter::SimpleBelt::SetFrisbeeSpeed(double speed) {
	m_motor->Set(speed);
}

void FrisbeeShooter::SimpleBelt::SetMaxDistance(float inches) {
	m_maxDistance = inches;
}
	
float FrisbeeShooter::SimpleBelt::GetMaxDistance() {
	return m_maxDistance;
}



FrisbeeShooter::ThreeWheel::ThreeWheel(SpeedController *frontMotor, SpeedController *middleMotor, SpeedController *lastMotor) :
		FrisbeeShooter::Base("FrisbeeShooter::ThreeWheel"),
		m_maxDistance(120) {
	m_frontMotor = frontMotor;
	m_middleMotor = middleMotor;
	m_lastMotor = lastMotor;
	AddActuatorToLiveWindow("Front Motor", m_frontMotor);
	AddActuatorToLiveWindow("Middle Motor", m_middleMotor);
	AddActuatorToLiveWindow("Last Motor", m_lastMotor);
}

FrisbeeShooter::ThreeWheel::~ThreeWheel() {
	// empty
}

void FrisbeeShooter::ThreeWheel::ShootFrisbee() {
	SetFrisbeeSpeed(1.0);
}

void FrisbeeShooter::ThreeWheel::ShootFrisbee(float distanceInInches) {
	// This needs calibration. For now, just do the default.
	ShootFrisbee();
}

void FrisbeeShooter::ThreeWheel::StopFrisbee() {
	SetFrisbeeSpeed(0);
}

void FrisbeeShooter::ThreeWheel::SetFrisbeeSpeed(double speed) {
	m_frontMotor->Set(speed);
	m_middleMotor->Set(speed);
	m_lastMotor->Set(speed);
}

void FrisbeeShooter::ThreeWheel::SetMaxDistance(float inches) {
	m_maxDistance = inches;
}

float FrisbeeShooter::ThreeWheel::GetMaxdistance() {
	return m_maxDistance;
}
