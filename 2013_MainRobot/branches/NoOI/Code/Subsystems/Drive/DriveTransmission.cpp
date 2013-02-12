#include "DriveTransmission.h"

BaseDriveTransmission::BaseDriveTransmission(const char *name) :
		BaseSubsystem(name) {
	// empty
}

BaseDriveTransmission::~BaseDriveTransmission() {
	// empty
}


SimpleDriveTransmission::SimpleDriveTransmission(Solenoid *leftPiston, Solenoid *rightPiston) :
		BaseDriveTransmission("SimpleDriveTransmision") {
	m_leftPiston = leftPiston;
	m_rightPiston = rightPiston;
	AddActuatorToLiveWindow("Left Solenoid", m_leftPiston);
	AddActuatorToLiveWindow("Right Solenoid", m_rightPiston);
}

SimpleDriveTransmission::~SimpleDriveTransmission() {
	// empty
}

void SimpleDriveTransmission::SetHighGear() {
	m_leftPiston->Set(true);
	m_rightPiston->Set(true);
}

void SimpleDriveTransmission::SetLowGear() {
	m_leftPiston->Set(false);
	m_rightPiston->Set(false);
}

void SimpleDriveTransmission::ToggleGear() {
	m_leftPiston->Set(!m_leftPiston->Get());
	m_rightPiston->Set(!m_rightPiston->Get());
}

BaseDriveTransmission::TransmissionMode SimpleDriveTransmission::GetCurrentMode() {
	bool left = m_leftPiston->Get();
	bool right = m_rightPiston->Get();
	
	if (left != right) {
		return kError;
	} else if (left) {
		return kHighGear;
	} else {
		return kLowGear;
	}
}
