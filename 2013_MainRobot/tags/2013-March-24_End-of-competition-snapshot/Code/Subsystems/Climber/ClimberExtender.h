#ifndef CLIMBER_EXTENDER_H
#define CLIMBER_EXTENDER_H
/**
 * \file ClimberExtender.h
 * 
 * \addtogroup subsystems
 * \{
 */
#include "WPILib.h"
#include "../BaseSubsystem.h"
#include "../../Misc/Tools.h"
#include"../../Misc/Ports.h"

namespace Extender {

/**
 * \brief Base class for climber extender.
 */
class Base : public BaseSubsystem {
public:
	Base(const char *name);
	virtual ~Base();
	virtual void PullPins() = 0;
};

/**
 * \brief Climber extender that uses pneumatics.
 */
class Pneumatic : public Base {
public:
	/**
	 * \param[in] rightSolenoid Pointer to solenoid on the right extender.
	 * \param[in] leftSolenoid Pointer to the solenoid on the left extender.
	 */
	Pneumatic(Solenoid *rightSolenoid, Solenoid *leftSolenoid);
	~Pneumatic();
	/**
	 * \brief Pulls pins to extend the extenders.
	 */
	void PullPins();
private:
	Solenoid *m_rightSolenoid;
	Solenoid *m_leftSolenoid;
};

}

/**
 * \}
 */

#endif
