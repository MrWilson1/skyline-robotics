#ifndef CLIMBER_COMMANDS_H
#define CLIMBER_COMMANDS_H
/**
 * \file ClimberCommands.h
 * 
 * \addtogroup commands
 * \{
 */

#include "BasicArmCommands.h"

#include "../Misc/Tools.h"
#include "math.h"

#include "WPILib.h"

#include "BaseCommand.h"
#include "../subsystems/Climber/ClimberExtender.h"
#include "../Subsystems/Controllers/Axis.h"

/**
 * Command to extend the arm.
 */
class ExtendArmCommand : public SimpleCommand {
public:
	ExtendArmCommand(BaseClimberExtender *climberExtender);
	~ExtendArmCommand();
	
	void Execute();
	
private:
	BaseClimberExtender *m_climberExtender;
};

/**
 * Command to hunk onto a rung.
 */
class HookOnToRungCommand : public CommandGroup {
public:
	HookOnToRungCommand (Arm arm, double heightOfRobot, double distanceBetweenShoulderAndRung);
	~HookOnToRungCommand(); 
};

/**
 * \brief Command to pull robot up.
 */
class PullRobotUpCommand : public CommandGroup {
public:
	PullRobotUpCommand(Arm arm);
	PullRobotUpCommand(Arm arm, double magnitude);
	~PullRobotUpCommand();
};

/**
 * Command to disengage the arm from a rung.
 */
class DisengageArmCommand : public CommandGroup {
public:
	DisengageArmCommand(Arm arm);
	~DisengageArmCommand();
private:
	double m_xDisplacement;
	double m_yDisplacement;
};

/**
 * Command to climb a whole level of the pyramid.
 */
class ClimbLevelCommand : public CommandGroup {
public:
	ClimbLevelCommand(Arm arm, double heightOfRobot, double distanceBetweenShoulderAndRung);
	~ClimbLevelCommand();
};

/**
 * Command to climb the entire pyramid.
 */
class ClimbPyramidCommand : public CommandGroup {
public:
	ClimbPyramidCommand(Arm arm, BaseClimberExtender *extender, double heightOfRobot, double distanceBetweenShoulderAndRung);
	~ClimbPyramidCommand();
};

/**
 * Command to control the elbow and shoulder of the arm manually by passing in Axes.
 */
class ControlArmManuallyCommand : public SimpleCommand {
public:
	ControlArmManuallyCommand(BaseJoint *elbow, BaseJoint *shoulder, Axis *elbowAxis, Axis *shoulderAxis);
	~ControlArmManuallyCommand();
	
	void Execute();
	
private:
	BaseJoint *m_elbow;
	BaseJoint *m_shoulder; 
	Axis *m_elbowAxis;
	Axis *m_shoulderAxis;
};

/**
 * \}
 */

#endif
