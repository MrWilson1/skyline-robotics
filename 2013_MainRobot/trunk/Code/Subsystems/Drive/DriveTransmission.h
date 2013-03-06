#ifndef DRIVE_TRANSMISSION_H
#define DRIVE_TRANSMISSION_H
/**
 * \file DriveTransmission.h
 * 
 * \brief Subsystems for the drive transmission (two-speed drive, etc).
 * 
 * Any subsystems in here should theoretically be able to run independently
 * of whatever [Drive](\ref drive_subsystems) is currently being used.
 * 
 * If a transmission is used, it's an extra thrown on top of a Drive.
 * 
 * \addtogroup subsystems
 * \{
 * \defgroup drive_transmission_subsystems Drive Transmissions
 * \{
 */
#include "WPILib.h"
#include "../BaseSubsystem.h"

/**
 * \brief A base class for any Drive Tranmission.
 * 
 * All transmissions should inherit from this.
 */
class BaseDriveTransmission : public BaseSubsystem {
public:
	// The names of these might need to be changed.
	enum TransmissionMode {
		kError, // when the two sides have different modes
		kHighGear,
		kLowGear
	};
	
	BaseDriveTransmission(const char *name);
	virtual ~BaseDriveTransmission();
	
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
	virtual TransmissionMode GetCurrentMode() = 0;
};

/**
 * \brief A simple transmission, using solenoids to switch between the gearings 
 */
class SimpleDriveTransmission : public BaseDriveTransmission {
public:
	SimpleDriveTransmission(DoubleSolenoid *leftPiston, DoubleSolenoid *rightPiston);
	~SimpleDriveTransmission();
	
	void SetHighGear();
	void SetLowGear();
	void ToggleGear();
	virtual TransmissionMode GetCurrentMode();
	
private:
	void Set();
	DoubleSolenoid *m_leftPiston;
	DoubleSolenoid *m_rightPiston;
	BaseDriveTransmission::TransmissionMode m_mode;
};


/**
 * \}
 * \}
 */
#endif
