#ifndef CLIMBER_COMMANDS_H
#define CLIMBER_COMMANDS_H

#include "WPILib.h"
#include "..\Misc\Tools.h"
#include "..\OperatorInterfaces\BaseOI.h"
#include "BaseCommand.h"
#include "..\subsystems\Climber\ClimberExtender.h"
#include "math.h"
#include "..\Subsystems\Controllers\Axis.h"
#include "BasicArmCommands.h"

/**
 * \brief Command to pull robot up.
 */
class PullRobotUpCommand : public CommandGroup {
public:
	PullRobotUpCommand(ArmData arm);
	PullRobotUpCommand(ArmData arm, double magnitude);
	~PullRobotUpCommand();
};

class ExtendArmCommand : public Command {
public:
	ExtendArmCommand(BaseClimberExtender *climberExtender);

	~ExtendArmCommand();
	
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
	
private:
	BaseClimberExtender *m_climberExtender;
};

class DisengageArmCommand : public CommandGroup {
public:
	DisengageArmCommand(ArmData arm);
	~DisengageArmCommand();
private:
	double m_xDisplacement;
	double m_yDisplacement;
};

class HookAndDisengageCommand : public CommandGroup {
public:
	HookAndDisengageCommand(ArmData arm, double heightOfRobot, double distanceBetweenShoulderAndRung);
	~HookAndDisengageCommand();
};

class HookOnToRungCommand : public CommandGroup {
public:
	HookOnToRungCommand (ArmData arm, double heightOfRobot, double distanceBetweenShoulderAndRung);
	~HookOnToRungCommand(); 
};

class ClimbPyramidCommand : public CommandGroup {
public:
	ClimbPyramidCommand(ArmData arm, BaseClimberExtender *extender, double heightOfRobot, double distanceBetweenShoulderAndRung);
	~ClimbPyramidCommand();
};

class ClimbLevelCommand : public CommandGroup {
public:
	ClimbLevelCommand(ArmData arm, double heightOfRobot, double distanceBetweenShoulderAndRung);
	~ClimbLevelCommand();
};

class ControlArmManuallyCommand : public Command {
public:
	ControlArmManuallyCommand(BaseJoint *elbow, BaseJoint *shoulder, Axis *elbowAxis, Axis *shoulderAxis);
	~ControlArmManuallyCommand();
	
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
private:
	BaseJoint *m_elbow;
	BaseJoint *m_shoulder; 
	Axis *m_elbowAxis;
	Axis *m_shoulderAxis;
};

#endif
