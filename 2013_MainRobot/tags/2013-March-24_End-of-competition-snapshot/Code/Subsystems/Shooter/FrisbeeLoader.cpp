#include "FrisbeeLoader.h"

FrisbeeLoader::Base::Base(const char *name) :
		BaseSubsystem(name) {
	// Empty
}

FrisbeeLoader::Base::~Base() {
	// Empty
}


FrisbeeLoader::Placeholder::Placeholder() :
		Base("Placeholder") {
	// empty
}

FrisbeeLoader::Placeholder::~Placeholder() {
	// empty
}

void FrisbeeLoader::Placeholder::PrepareFrisbee() {
	// empty
}

bool FrisbeeLoader::Placeholder::IsFrisbeePrepared() {
	return true;
}

void FrisbeeLoader::Placeholder::LoadFrisbee() {
	// empty
}



FrisbeeLoader::Piston::Piston(Solenoid *piston) :
		Base("Piston") {
	m_piston = piston;
	
	AddActuatorToLiveWindow("Piston", m_piston);
}

FrisbeeLoader::Piston::~Piston() {
	//
}

void FrisbeeLoader::Piston::PrepareFrisbee() {
	// empty
}

bool FrisbeeLoader::Piston::IsFrisbeePrepared() {
	return true;
}

void FrisbeeLoader::Piston::LoadFrisbee() {
	m_piston->Set(true);
	Wait(0.1); // Otherwise, the piston doesn't move. 
	m_piston->Set(false);
}



FrisbeeLoader::ServoLoader::ServoLoader(Servo *servo) :
		Base("ServoLoader") {
	m_servo = servo;
	
	AddActuatorToLiveWindow("Servo", m_servo);
}

FrisbeeLoader::ServoLoader::~ServoLoader() {
	//
}

void FrisbeeLoader::ServoLoader::PrepareFrisbee() {
	// empty
}

bool FrisbeeLoader::ServoLoader::IsFrisbeePrepared() {
	return true;
}

void FrisbeeLoader::ServoLoader::LoadFrisbee() {
	SmartDashboard::PutString(GetName(), "Requires calibration");
	// I'm just guessing values here.
	m_servo->SetAngle(180);
	m_servo->SetAngle(0);
}
