#include "BaseSubsystem.h"

BaseSubsystem::BaseSubsystem(const char *name) :
		Subsystem(name) {
	// empty
}

BaseSubsystem::~BaseSubsystem() {
	// empty
}



void BaseSubsystem::ReportLiveWindowError(char *name) {
	std::string errorKey;
	errorKey.append(GetName());
	errorKey.append(": ");
	errorKey.append(name);
	std::string errorMessage("Could not convert to LiveWindowSendable");
	SmartDashboard::PutString(errorKey, errorMessage);
}
