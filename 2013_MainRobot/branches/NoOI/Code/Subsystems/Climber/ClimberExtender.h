#ifndef CLIMBER_EXTENDER_H
#define CLIMBER_EXTENDER_H

#include "WPILib.h"
#include "../BaseSubsystem.h"
#include "../../Misc/Tools.h"
#include"../../Misc/Ports.h"

/**
 * \brief Base class for climber extender.
 */
class BaseClimberExtender : public BaseSubsystem {
public:
	BaseClimberExtender(const char *name);
	virtual ~BaseClimberExtender();
	virtual void PullPins() = 0;
};

/**
 * \brief Climber extender that uses pneumatics.
 */
class PneumaticClimberExtender : public BaseClimberExtender {
public:
	/**
	 * \param[in] rightSolenoid Pointer to solenoid on the right extender.
	 * \param[in] leftSolenoid Pointer to the solenoid on the left extender.
	 */
	PneumaticClimberExtender(Solenoid *rightSolenoid, Solenoid *leftSolenoid);
	~PneumaticClimberExtender();
	/**
	 * \brief Pulls pins to extend the extenders.
	 */
	void PullPins();
private:
	Solenoid *m_rightSolenoid;
	Solenoid *m_leftSolenoid;
};

#endif
