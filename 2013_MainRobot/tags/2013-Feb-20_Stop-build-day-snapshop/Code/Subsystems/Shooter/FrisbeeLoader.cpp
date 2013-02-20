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

int PlaceholderFrisbeeLoader::GetNumberOfFrisbeesLoaded() {
	return 1;
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

int PistonFrisbeeLoader::GetNumberOfFrisbeesLoaded() {
	return 4;
}

void PistonFrisbeeLoader::PrepareFrisbee() {
	// empty
}

bool PistonFrisbeeLoader::IsFrisbeePrepared() {
	return true;
}

void PistonFrisbeeLoader::LoadFrisbee() {
	m_piston->Set(true); // out
	m_piston->Set(false); // in
}
