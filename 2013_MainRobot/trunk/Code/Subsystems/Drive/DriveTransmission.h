#ifndef DRIVE_TRANSMISSION_H
#define DRIVE_TRANSMISSION_H
/**
 * \file DriveTransmission.h
 * 
 * \addtogroup subsystems
 * \{
 */
#include "WPILib.h"
#include "../BaseSubsystem.h"

/**
 * \brief Subsystems for the drive transmission (two-speed drive, etc).
 * 
 * Any subsystems in here should theoretically be able to run independently
 * of whatever [Drive](\ref drive_subsystems) is currently being used.
 * 
 * If a transmission is used, it's an extra thrown on top of a Drive.
 */
namespace DriveTransmission {

// The names of these might need to be changed.
enum Mode {
	kError, // when the two sides have different modes
	kHighGear,
	kLowGear
};

/**
 * \brief A base class for any Drive Tranmission.
 * 
 * All transmissions should inherit from this.
 */
class Base : public BaseSubsystem {
public:
	Base(const char *name);
	virtual ~Base();
	
	/**
	 * \brief Makes the robot have more maneuverability
	 */
	virtual void SetHighGear() = 0;
	
	/**
	 * \brief Makes the robot more powerful
	 */
	virtual void SetLowGear() = 0;
	
	/**
	 * \brief Toggles the transmission to whatever the opposite setting is.
	 */
	virtual void ToggleGear() = 0;
	
	/**
	 * \brief Gets the current transmission mode.
	 */
	virtual Mode GetCurrentMode() = 0;
};

/**
 * \brief A simple transmission, using solenoids to switch between the gearings 
 */
class Simple : public Base {
public:
	Simple(DoubleSolenoid *leftPiston, DoubleSolenoid *rightPiston);
	~Simple();
	
	void SetHighGear();
	void SetLowGear();
	void ToggleGear();
	virtual Mode GetCurrentMode();
	
private:
	void Set();
	DoubleSolenoid *m_leftPiston;
	DoubleSolenoid *m_rightPiston;
	Mode m_mode;
};

}

/**
 * \}
 */
#endif
