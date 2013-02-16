#include "DriveTransmission.h"

BaseDriveTransmission::BaseDriveTransmission(const char *name) :
		BaseSubsystem(name) {
	// empty
}

BaseDriveTransmission::~BaseDriveTransmission() {
	// empty
}


SimpleDriveTransmission::SimpleDriveTransmission(DoubleSolenoid *leftPiston, DoubleSolenoid *rightPiston) :
		BaseDriveTransmission("SimpleDriveTransmision"),
		m_mode(BaseDriveTransmission::kHighGear){
	m_leftPiston = leftPiston;
	m_rightPiston = rightPiston;
	AddActuatorToLiveWindow("Left Solenoid", m_leftPiston);
	AddActuatorToLiveWindow("Right Solenoid", m_rightPiston);
}

SimpleDriveTransmission::~SimpleDriveTransmission() {
	// empty
}

void SimpleDriveTransmission::SetHighGear() {
	m_leftPiston->Set(DoubleSolenoid::kForward);
	m_rightPiston->Set(DoubleSolenoid::kForward);
	m_mode = BaseDriveTransmission::kHighGear;
}

void SimpleDriveTransmission::SetLowGear() {
	m_leftPiston->Set(DoubleSolenoid::kReverse);
	m_rightPiston->Set(DoubleSolenoid::kReverse);
	m_mode = BaseDriveTransmission::kLowGear;
}

void SimpleDriveTransmission::ToggleGear() {
	if (m_mode == BaseDriveTransmission::kHighGear) {
		m_leftPiston->Set(DoubleSolenoid::kReverse);
		m_rightPiston->Set(DoubleSolenoid::kReverse);
		m_mode = BaseDriveTransmission::kLowGear;
	} else if (m_mode == BaseDriveTransmission::kLowGear) {
		m_leftPiston->Set(DoubleSolenoid::kForward);
		m_rightPiston->Set(DoubleSolenoid::kForward);
		m_mode = BaseDriveTransmission::kHighGear;
	} else {
		SmartDashboard::PutString(GetName() + std::string(" Error"), "Unknown mode");
	}
}

BaseDriveTransmission::TransmissionMode SimpleDriveTransmission::GetCurrentMode() {
	return m_mode;
}
