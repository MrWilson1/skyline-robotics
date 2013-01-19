#include "FrisbeeAimer.h"


BaseFrisbeeAimer::BaseFrisbeeAimer(const char *name) :
		BaseSubsystem(name) {
	// Empty
}

BaseFrisbeeAimer::~BaseFrisbeeAimer() {
	// Empty
}

Offset::Offset(float x, float y) :
		XOffset(x), 
		YOffset(y) {
	// Empty
}
