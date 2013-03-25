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

void BaseSubsystem::ReportNullPointerError(char *name) {
	std::string errorKey;
	errorKey.append(GetName());
	errorKey.append(": ");
	errorKey.append(name);
	std::string errorMessage("Object is null -- was it initialized?");
	SmartDashboard::PutString(errorKey, errorMessage);
}
