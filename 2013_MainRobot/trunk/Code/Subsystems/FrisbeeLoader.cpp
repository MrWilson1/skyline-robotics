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

void PlaceholderFrisbeeLoader::UnloadFrisbee() {
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
