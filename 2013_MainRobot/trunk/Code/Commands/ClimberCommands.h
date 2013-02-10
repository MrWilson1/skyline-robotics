#ifndef CLIMBER_COMMANDS_H
#define CLIMBER_COMMANDS_H

#include "WPILib.h"
#include "..\Misc\Tools.h"
#include "..\OperatorInterfaces\BaseOI.h"
#include "BaseCommand.h"
#include "..\Subsystems\Climber\ClimberArm.h"
#include "..\subsystems\Climber\ClimberExtender.h"

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

#endif
