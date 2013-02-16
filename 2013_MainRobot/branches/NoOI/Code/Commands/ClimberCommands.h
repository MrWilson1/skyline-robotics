#ifndef CLIMBER_COMMANDS_H
#define CLIMBER_COMMANDS_H

#include "WPILib.h"
#include "..\Misc\Tools.h"
#include "..\OperatorInterfaces\BaseOI.h"
#include "BaseCommand.h"
#include "..\Subsystems\Climber\ClimberArm.h"
#include "..\subsystems\Climber\ClimberExtender.h"
#include "math.h"

class PullRobotUpCommand : public Command {
public:
	PullRobotUpCommand(BaseClimberArm *arm);
	PullRobotUpCommand(BaseClimberArm *arm, float magnitude);
	~PullRobotUpCommand();
	
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
		
private: 
	BaseClimberArm *m_arm; 	
	float m_magnitude;
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

class MoveArmCartesianCommand : public Command {
public:
	MoveArmCartesianCommand(BaseClimberArm *arm, double newX, double newY);
	~MoveArmCartesianCommand();
	
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();

private:
	BaseClimberArm *m_arm;
	double m_newX;
	double m_newY;
	double range;
};

class MoveArmPolarCommand : public Command {
public:
	MoveArmPolarCommand(BaseClimberArm *arm, double newAngle, double newMagnitude);
	~MoveArmPolarCommand();
	
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
	
private:
	BaseClimberArm *m_arm;
	double m_newAngle;
	double m_newMagnitude;
	double m_angleRange;
	double m_magnitudeRange;
};

class DisengageArmCommand : public CommandGroup {
public:
	DisengageArmCommand(BaseClimberArm *arm);
	~DisengageArmCommand();
private:
	double m_xDisplacement;
	double m_yDisplacement;
};

class HookAndDisengageCommand : public CommandGroup {
public:
	HookAndDisengageCommand(BaseClimberArm *arm, double heightOfRobot, double distanceBetweenShoulderAndRung);
	~HookAndDisengageCommand();
};

class HookOnToRungCommand : public CommandGroup {
public:
	HookOnToRungCommand (BaseClimberArm *arm, double heightOfRobot, double distanceBetweenShoulderAndRung);
	~HookOnToRungCommand(); 
	
private:
	BaseClimberArm *m_arm;  
};
#endif
