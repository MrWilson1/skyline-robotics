#include "FrisbeeShooter.h"

FrisbeeShooter::Base::Base(const char *name) :
		BaseSubsystem(name) {
	// Empty
}

FrisbeeShooter::Base::~Base() {
	// Empty
}




FrisbeeShooter::SimpleBelt::SimpleBelt(SpeedController *motor) :
		FrisbeeShooter::Base("FrisbeeShooter::SimpleBelt") {
	m_motor = motor;
	
	AddActuatorToLiveWindow("Belt motor", m_motor);
}

FrisbeeShooter::SimpleBelt::~SimpleBelt() {
	// Empty
}

void FrisbeeShooter::SimpleBelt::ShootFrisbee() {
	m_motor->Set(1.0);
}

void FrisbeeShooter::SimpleBelt::ShootFrisbee(float inchesPerSecond) {
	// Not implemented
	m_motor->Set(0.0);
}

void FrisbeeShooter::SimpleBelt::StopFrisbee() {
	m_motor->Set(0);
}

void FrisbeeShooter::SimpleBelt::SetFrisbeeSpeed(double speed) {
	m_motor->Set(speed);
}




FrisbeeShooter::ThreeWheel::ThreeWheel(SpeedController *frontMotor, SpeedController *middleMotor, SpeedController *lastMotor) :
		FrisbeeShooter::Base("FrisbeeShooter::ThreeWheel") {
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
	SmartDashboard::PutNumber("ThreeWheel raw", speed);
}





FrisbeeShooter::ThreeWheelSteady::ThreeWheelSteady(
		SpeedController *frontMotor, 
		SpeedController *middleMotor, 
		SpeedController *lastMotor,
		Encoder *encoder,
		float maxEncoderPulses,
		float maxEncoderSpeed) :
		FrisbeeShooter::Base("FrisbeeShooter::ThreeWheel"),
		m_maxEncoderPulses(maxEncoderPulses),
		m_maxEncoderSpeed(maxEncoderSpeed),
		m_targetSpeed(0),
		m_outputSpeed(0) {
	m_frontMotor = frontMotor;
	m_middleMotor = middleMotor;
	m_lastMotor = lastMotor;
	m_encoder = encoder;
	m_encoder->Start();
	AddActuatorToLiveWindow("Front Motor", m_frontMotor);
	AddActuatorToLiveWindow("Middle Motor", m_middleMotor);
	AddActuatorToLiveWindow("Last Motor", m_lastMotor);
	AddActuatorToLiveWindow("Encoder", m_encoder);
}

FrisbeeShooter::ThreeWheelSteady::~ThreeWheelSteady() {
	// empty
}

void FrisbeeShooter::ThreeWheelSteady::ShootFrisbee() {
	SetFrisbeeSpeed(1.0);
	m_targetSpeed = 1.0;
}

void FrisbeeShooter::ThreeWheelSteady::ShootFrisbee(float inchesPerSecond) {
	double rawSpeed = inchesPerSecond / m_maxEncoderSpeed;
	SetFrisbeeSpeed(rawSpeed);
}

void FrisbeeShooter::ThreeWheelSteady::StopFrisbee() {
	// manual override.
	m_frontMotor->Set(0);
	m_middleMotor->Set(0);
	m_lastMotor->Set(0);
	m_targetSpeed = 0;
	m_outputSpeed = 0;
}

void FrisbeeShooter::ThreeWheelSteady::SetFrisbeeSpeed(double speed) {
	m_targetSpeed = speed;
	double m_reportedSpeed = m_encoder->GetRate() / m_maxEncoderPulses;
	if (Tools::IsWithinRange(m_reportedSpeed, m_targetSpeed, 0.015)) {
		// nothing.
	} else if (m_reportedSpeed < m_targetSpeed) {
		m_outputSpeed += 0.02;
	} else if (m_reportedSpeed > m_targetSpeed) {
		m_outputSpeed -= 0.02;
	}
	
	m_frontMotor->Set(m_outputSpeed);
	m_middleMotor->Set(m_outputSpeed);
	m_lastMotor->Set(m_outputSpeed);
}

