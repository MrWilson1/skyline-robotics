#ifndef CLIMBER_EXTENDER_H
#define CLIMBER_EXTENDER_H

#include "WPILib.h"
#include "../BaseSubsystem.h"
#include "../../Misc/Tools.h"
#include"../../Misc/Ports.h"

class BaseClimberExtender : public BaseSubsystem {
public:
	BaseClimberExtender(const char *name);
	virtual ~BaseClimberExtender();
	virtual void PullPins() = 0;
};

class PneumaticClimberExtender : public BaseClimberExtender {
public:
	PneumaticClimberExtender(Solenoid *rightSolenoid, Solenoid *leftSolenoid);
	~PneumaticClimberExtender();
	void PullPins();
private:
	Solenoid *m_rightSolenoid;
	Solenoid *m_leftSolenoid;
};

#endif
