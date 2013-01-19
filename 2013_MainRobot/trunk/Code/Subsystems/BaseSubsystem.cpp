#include "BaseSubsystem.h"

BaseSubsystem::BaseSubsystem(const char *name) :
		Subsystem(name) {
	// empty
}

BaseSubsystem::~BaseSubsystem() {
	// empty
}

template<class T>
void BaseSubsystem::AddActuatorToLiveWindow(char *name, T *object) {
	LiveWindow *lw = LiveWindow::GetInstance();
	LiveWindowSendable *sendable = dynamic_cast<LiveWindowSendable *>(object);
	if (sendable == NULL) {
		ReportLiveWindowError(name);
	} else {
		lw->AddActuator((char *) GetName().c_str(), name, sendable);
	}
}

template<class T>
void BaseSubsystem::AddSensorToLiveWindow(char *name, T *object) {
	LiveWindow *lw = LiveWindow::GetInstance();
	LiveWindowSendable *sendable = dynamic_cast<LiveWindowSendable *>(object);
	if (sendable == NULL) {
		ReportLiveWindowError(name);
	} else {
		lw->AddSensor((char *) GetName().c_str(), name, sendable);
	}
}

template<class T>
void BaseSubsystem::AddComponentToLiveWindow(char *name, T *object) {
	LiveWindow *lw = LiveWindow::GetInstance();
	LiveWindowSendable *sendable = dynamic_cast<LiveWindowSendable *>(object);
	if (sendable == NULL) {
		ReportLiveWindowError(name);
	} else {
		lw->AddComponent((char *) GetName().c_str(), name, sendable);
	}
}

void BaseSubsystem::ReportLiveWindowError(char *name) {
	std::string errorKey;
	errorKey.append(GetName());
	errorKey.append(": ");
	errorKey.append(name);
	std::string errorMessage("Could not convert to LiveWindowSendable");
	SmartDashboard::PutString(errorKey, errorMessage);
}
