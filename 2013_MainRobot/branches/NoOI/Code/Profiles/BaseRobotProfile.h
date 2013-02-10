#ifndef BASE_ROBOT_PROFILE_H
#define BASE_ROBOT_PROFILE_H

#include <string>

#include "WPILib.h"

/**
 * Defines a base robot profile. All robot profiles should
 * be child classes of this one.
 * 
 * This class will automatically call the scheduler for events
 * occurring during the Autonomous, Teleop, and Test periodic
 * modes. 
 * 
 * The child classes must implement 
 */
class BaseRobotProfile : public IterativeRobot {
public:
	BaseRobotProfile(const char *name);
	virtual ~BaseRobotProfile();
	
	// You must override these (mandated by us)
	virtual void CreateBasicHardwareObjects() = 0;
	virtual void CreateSubsystems() = 0;
	virtual void CreateOI() = 0;
	
	// You must override these (mandated by FIRST)
	virtual void RobotInit() = 0;
	virtual void AutonomousInit() = 0;
	virtual void TeleopInit() = 0;
	
	// These will be predefined since they'll be identical
	// for each robot (but can be overridden if necessary)
	virtual void AutonomousPeriodic();
	virtual void TeleopPeriodic();
	virtual void TestPeriodic();
	
	virtual std::string GetName();
	
protected:
	// Internal maintenance stuff (but child classes
	// should take advantage of LiveWindow)
	LiveWindow *m_liveWindow;
	const std::string m_name;
};


#endif
