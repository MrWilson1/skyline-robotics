#include "FrisbeeLoader.h"

BaseFrisbeeLoader::BaseFrisbeeLoader(const char *name) :
		BaseSubsystem(name) {
	// Empty
}

BaseFrisbeeLoader::~BaseFrisbeeLoader() {
	// Empty
}


PlaceholderFrisbeeLoader::PlaceholderFrisbeeLoader() :
		BaseFrisbeeLoader("PlaceholderFrisbeeLoader") {
	// empty
}

PlaceholderFrisbeeLoader::~PlaceholderFrisbeeLoader() {
	// empty
}

void PlaceholderFrisbeeLoader::PrepareFrisbee() {
	// empty
}

bool PlaceholderFrisbeeLoader::IsFrisbeePrepared() {
	return true;
}

void PlaceholderFrisbeeLoader::LoadFrisbee() {
	// empty
}



PistonFrisbeeLoader::PistonFrisbeeLoader(Solenoid *piston) :
		BaseFrisbeeLoader("PistonFrisbeeLoader") {
	m_piston = piston;
	
	AddActuatorToLiveWindow("Piston", m_piston);
}

PistonFrisbeeLoader::~PistonFrisbeeLoader() {
	//
}

void PistonFrisbeeLoader::PrepareFrisbee() {
	// empty
}

bool PistonFrisbeeLoader::IsFrisbeePrepared() {
	return true;
}

void PistonFrisbeeLoader::LoadFrisbee() {
	m_piston->Set(false);
	m_piston->Set(true);
}



ServoFrisbeeLoader::ServoFrisbeeLoader(Servo *servo) :
		BaseFrisbeeLoader("ServoFrisbeeLoader") {
	m_servo = servo;
	
	AddActuatorToLiveWindow("Servo", m_servo);
}

ServoFrisbeeLoader::~ServoFrisbeeLoader() {
	//
}

void ServoFrisbeeLoader::PrepareFrisbee() {
	// empty
}

bool ServoFrisbeeLoader::IsFrisbeePrepared() {
	return true;
}

void ServoFrisbeeLoader::LoadFrisbee() {
	SmartDashboard::PutString(GetName(), "Requires calibration");
	// I'm just guessing values here.
	m_servo->SetAngle(180);
	m_servo->SetAngle(0);
}
