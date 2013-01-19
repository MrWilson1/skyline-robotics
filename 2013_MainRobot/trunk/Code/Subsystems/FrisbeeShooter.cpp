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

void SimpleBeltShooter::SetMaxDistance(float inches) {
	m_maxDistance = inches;
}
	
float SimpleBeltShooter::GetMaxDistance() {
	return m_maxDistance;
}
