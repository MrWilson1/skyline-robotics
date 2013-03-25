#include "DriveTransmission.h"

DriveTransmission::Base::Base(const char *name) :
		BaseSubsystem(name) {
	// empty
}

DriveTransmission::Base::~Base() {
	// empty
}


DriveTransmission::Simple::Simple(DoubleSolenoid *leftPiston, DoubleSolenoid *rightPiston) :
		DriveTransmission::Base("DriveTransmision::Simple"),
		m_mode(DriveTransmission::kHighGear){
	m_leftPiston = leftPiston;
	m_rightPiston = rightPiston;
	AddActuatorToLiveWindow("Left Solenoid", m_leftPiston);
	AddActuatorToLiveWindow("Right Solenoid", m_rightPiston);
}

DriveTransmission::Simple::~Simple() {
	// empty
}

void DriveTransmission::Simple::SetHighGear() {
	m_leftPiston->Set(DoubleSolenoid::kForward);
	m_rightPiston->Set(DoubleSolenoid::kForward);
	m_mode = DriveTransmission::kHighGear;
}

void DriveTransmission::Simple::SetLowGear() {
	m_leftPiston->Set(DoubleSolenoid::kReverse);
	m_rightPiston->Set(DoubleSolenoid::kReverse);
	m_mode = DriveTransmission::kLowGear;
}

void DriveTransmission::Simple::ToggleGear() {
	if (m_mode == DriveTransmission::kHighGear) {
		m_leftPiston->Set(DoubleSolenoid::kReverse);
		m_rightPiston->Set(DoubleSolenoid::kReverse);
		m_mode = DriveTransmission::kLowGear;
	} else if (m_mode == DriveTransmission::kLowGear) {
		m_leftPiston->Set(DoubleSolenoid::kForward);
		m_rightPiston->Set(DoubleSolenoid::kForward);
		m_mode = DriveTransmission::kHighGear;
	} else {
		SmartDashboard::PutString(GetName() + std::string(" Error"), "Unknown mode");
	}
}

DriveTransmission::Mode DriveTransmission::Simple::GetCurrentMode() {
	return m_mode;
}
