#ifndef BASE_SUBSYSTEM_H
#define BASE_SUBSYSTEM_H

#include <string>
#include "WPILib.h"

class BaseSubsystem : public Subsystem {
public:
	BaseSubsystem(const char *name);
	virtual ~BaseSubsystem();
	
	// The following are equivalent to
	// 
	//     LiveWindow::GetInstance()->AddActuator
	//     LiveWindow::GetInstance()->AddSensor
	//     LiveWindow::GetInstance()->AddComponent
	//
	// ...except they automatically specify the subsystem and do
	// some basic error checking.
	//
	// Nearly all WPILib objects inherit from ErrorBase.
	//
	// I don't actually know what a 'Component' is, so don't use it unless
	// you find out what it is.
	template<class T>
	void AddActuatorToLiveWindow(char *name, T *object);
	
	template<class T>
	void AddSensorToLiveWindow(char *name, T *object);
	
	template<class T>
	void AddComponentToLiveWindow(char *name, T *object);
	
protected:
	void ReportLiveWindowError(char *name);
};

#endif
