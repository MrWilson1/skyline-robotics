#ifndef SHOOTER_COMMANDS_H
#define SHOOTER_COMMANDS_H

#include "WPILib.h"
#include "..\Misc\Tools.h"
#include "..\OperatorInterfaces\BaseOI.h"

#include "..\Subsystems\FrisbeeAimer.h"
#include "..\Subsystems\FrisbeeLoader.h"
#include "..\Subsystems\FrisbeeShooter.h"
#include "..\Subsystems\FrisbeeTurret.h"

class LoadFrisbeeCommand : Command {
public:
	LoadFrisbeeCommand(BaseFrisbeeLoader *loader);
	~LoadFrisbeeCommand();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
	
private:
	BaseFrisbeeLoader *m_loader;
	bool m_isFinished;
};

class AimTurretCommand : Command {
public:
	AimTurretCommand(BaseFrisbeeAimer *aimer);
	~AimTurretCommand();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
	
private:
	BaseFrisbeeAimer *m_aimer;
};

class FireFrisbeeCommand : Command {
public:
	FireFrisbeeCommand(BaseFrisbeeShooter *shooter);
	FireFrisbeeCommand(BaseFrisbeeShooter *shooter, double distanceInInches);
	~FireFrisbeeCommand();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
		
private:
	BaseFrisbeeShooter *m_shooter;
	double m_distanceInInches;
};


class EjectFrisbeeCommand : Command {
public:
	EjectFrisbeeCommand(BaseFrisbeeShooter *shooter);
	~EjectFrisbeeCommand();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
		
private:
	BaseFrisbeeShooter *m_shooter;
};

#endif
