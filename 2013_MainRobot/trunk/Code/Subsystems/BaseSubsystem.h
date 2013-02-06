/**
 * \file BaseSubsystem.h
 * 
 * \brief Extends the WPILib Subsystem class
 * 
 * \defgroup subsystems Subsystems
 * 
 *  A class which controls a specific component on the robot.
 *  
 *  Each subsystem should inherit from some base, abstract class
 *  (which inherits from [BaseSubsystem](\ref BaseSubsystem).
 *  
 *  \addtogroup subsystems
 *  \{
 */

#ifndef BASE_SUBSYSTEM_H
#define BASE_SUBSYSTEM_H

#include <string>
#include "WPILib.h"

/**
 * \brief The abstract base class for all subsystems.
 * 
 * Provides additional functionality to automatically dynamically
 * cast arbitrary actuators and sensors and add them to the LiveWindow.
 * 
 * The [AddActuatorToLiveWindow](\ref AddActuatorToLiveWindow) and 
 * [AddSensorToLiveWindow](\ref AddSensorToLiveWindow) are equivalent
 * to calling:
 * 
 *     LiveWindow::GetInstance()->AddActuator();
 *     LiveWindow::GetInstance()->AddSensor();
 *     
 * ...except it properly dynamically casts the objects and do some
 * basic error checking.
 * 
 * All objects are first dynamically cast to ErrorBase, since they
 * nearly all inherit from it.
 */
class BaseSubsystem : public Subsystem {
public:
	/**
	 * \param[in] name The name of the subsystem (can be any arbitrary string).
	 * Used to display a nice name on the SmartDashboard.
	 */
	BaseSubsystem(const char *name);
	virtual ~BaseSubsystem();
	
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
	/**
	 * \brief Reports an error to SmartDashboard
	 */
	void ReportLiveWindowError(char *name);
};

/** \} */
#endif
