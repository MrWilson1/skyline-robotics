#ifndef CLIMBER_COMMANDS_H
#define CLIMBER_COMMANDS_H

#include "BasicArmCommands.h"

#include "../Misc/Tools.h"
#include "math.h"

#include "WPILib.h"

#include "BaseCommand.h"
#include "../subsystems/Climber/ClimberExtender.h"
#include "../Subsystems/Controllers/Axis.h"


class ExtendArmCommand : public SimpleCommand {
public:
	ExtendArmCommand(BaseClimberExtender *climberExtender);
	~ExtendArmCommand();
	
	void Execute();
	
private:
	BaseClimberExtender *m_climberExtender;
};

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

class DisengageArmCommand : public CommandGroup {
public:
	DisengageArmCommand(Arm arm);
	~DisengageArmCommand();
private:
	double m_xDisplacement;
	double m_yDisplacement;
};

class ClimbLevelCommand : public CommandGroup {
public:
	ClimbLevelCommand(Arm arm, double heightOfRobot, double distanceBetweenShoulderAndRung);
	~ClimbLevelCommand();
};

class ClimbPyramidCommand : public CommandGroup {
public:
	ClimbPyramidCommand(Arm arm, BaseClimberExtender *extender, double heightOfRobot, double distanceBetweenShoulderAndRung);
	~ClimbPyramidCommand();
};

class ControlArmManuallyCommand : public SimpleCommand {
public:
	ControlArmManuallyCommand(Joint::Base *elbow, Joint::Base *shoulder, Axis *elbowAxis, Axis *shoulderAxis);
	~ControlArmManuallyCommand();
	
	void Execute();
	
private:
	Joint::Base *m_elbow;
	Joint::Base *m_shoulder; 
	Axis *m_elbowAxis;
	Axis *m_shoulderAxis;
};

#endif
