#ifndef BASE_SUBSYSTEM_H
#define BASE_SUBSYSTEM_H

#include <string>
#include "WPILib.h"

class BaseSubsystem : public Subsystem {
public:
	BaseSubsystem(const char *name);
	virtual ~BaseSubsystem();
	
	/** The following are equivalent to
	 * 
	 *     LiveWindow::GetInstance()->AddActuator
	 *     LiveWindow::GetInstance()->AddSensor
	 *
	 * ...except they automatically specify the subsystem and do
	 * some basic error checking.
	 *
	 * Nearly all WPILib objects inherit from ErrorBase.
	 */
	template<class T>
	void AddActuatorToLiveWindow(char *name, T *object) {
		LiveWindow *lw = LiveWindow::GetInstance();
		LiveWindowSendable *sendable = dynamic_cast<LiveWindowSendable *>(object);
		if (sendable == NULL) {
			ReportLiveWindowError(name);
		} else {
			lw->AddActuator((char *) GetName().c_str(), name, sendable);
		}
	}

	template<class T>
	void AddSensorToLiveWindow(char *name, T *object) {
		LiveWindow *lw = LiveWindow::GetInstance();
		LiveWindowSendable *sendable = dynamic_cast<LiveWindowSendable *>(object);
		if (sendable == NULL) {
			ReportLiveWindowError(name);
		} else {
			lw->AddSensor((char *) GetName().c_str(), name, sendable);
		}
	}
	
protected:
	void ReportLiveWindowError(char *name);
};

#endif
