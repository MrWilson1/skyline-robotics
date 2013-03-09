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

namespace ClimberCommand {
/**
 * Command to extend the arm.
 */
class ExtendArm :  public SimpleCommand {
public:
	ExtendArm(BaseClimberExtender *climberExtender);
	~ExtendArm();
	
	void Execute();
	
private:
	BaseClimberExtender *m_climberExtender;
};

/**
 * Command to hunk onto a rung.
 */
class HookOnToRung :  public CommandGroup {
public:
	HookOnToRung(BasicArmCommand::Arm arm, double heightOfRobot, double distanceBetweenShoulderAndRung);
	~HookOnToRung(); 
};

/**
 * \brief Command to pull robot up.
 */
class PullRobotUp :  public CommandGroup {
public:
	PullRobotUp(BasicArmCommand::Arm arm);
	PullRobotUp(BasicArmCommand::Arm arm, double magnitude);
	~PullRobotUp();
};

/**
 * Command to disengage the arm from a rung.
 */
class DisengageArm :  public CommandGroup {
public:
	DisengageArm(BasicArmCommand::Arm arm);
	~DisengageArm();
private:
	double m_xDisplacement;
	double m_yDisplacement;
};

/**
 * Command to climb a whole level of the pyramid.
 */
class ClimbLevel :  public CommandGroup {
public:
	ClimbLevel(BasicArmCommand::Arm arm, double heightOfRobot, double distanceBetweenShoulderAndRung);
	~ClimbLevel();
};

/**
 * Command to climb the entire pyramid.
 */
class ClimbPyramid :  public CommandGroup {
public:
	ClimbPyramid(BasicArmCommand::Arm arm, BaseClimberExtender *extender, double heightOfRobot, double distanceBetweenShoulderAndRung);
	~ClimbPyramid();
};

/**
 * Command to control the elbow and shoulder of the arm manually by passing in Axes.
 */
class ControlArmManually :  public SimpleCommand {
public:
	ControlArmManually(BaseJoint *elbow, BaseJoint *shoulder, Axis *elbowAxis, Axis *shoulderAxis);
	~ControlArmManually();
	
	void Execute();
	
private:
	BaseJoint *m_elbow;
	BaseJoint *m_shoulder; 
	Axis *m_elbowAxis;
	Axis *m_shoulderAxis;
};
}

/**
 * \}
 */

#endif
