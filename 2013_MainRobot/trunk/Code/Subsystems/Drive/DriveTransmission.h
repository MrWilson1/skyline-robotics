#ifndef DRIVE_TRANSMISSION_H
#define DRIVE_TRANSMISSION_H

#include "WPILib.h"
#include "../BaseSubsystem.h"

class BaseDriveTransmission : public BaseSubsystem {
public:
	// The names of these might need to be changed.
	enum TransmissionMode {
		Error, // when the two sides have different modes
		HighGear,
		LowGear
	};
	
	BaseDriveTransmission(const char *name);
	virtual ~BaseDriveTransmission();
	
	virtual void SetHighGear() = 0;
	virtual void SetLowGear() = 0;
	virtual TransmissionMode GetCurrentMode() = 0;
};


class SimpleDriveTransmission : public BaseDriveTransmission {
public:
	SimpleDriveTransmission(Solenoid *leftPiston, Solenoid *rightPiston);
	~SimpleDriveTransmission();
	
	void SetHighGear();
	void SetLowGear();
	virtual TransmissionMode GetCurrentMode();
	
private:
	Solenoid *m_leftPiston;
	Solenoid *m_rightPiston;
};


#endif
