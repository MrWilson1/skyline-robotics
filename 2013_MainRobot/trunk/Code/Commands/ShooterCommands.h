#ifndef SHOOTER_COMMANDS_H
#define SHOOTER_COMMANDS_H

#include "WPILib.h"
#include "..\Misc\Tools.h"
#include "..\OperatorInterfaces\BaseOI.h"

#include "..\Subsystems\Shooter\FrisbeeAimer.h"
#include "..\Subsystems\Shooter\FrisbeeLoader.h"
#include "..\Subsystems\Shooter\FrisbeeShooter.h"
#include "..\Subsystems\Shooter\FrisbeeTurret.h"

class LoadFrisbeeCommand : public Command {
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

class AimTurretCommand : public Command {
public:
	AimTurretCommand(BaseFrisbeeAimer *aimer, BaseFrisbeeTurret *turret, float allowedRange);
	~AimTurretCommand();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
	
private:
	BaseFrisbeeAimer *m_aimer;
	BaseFrisbeeTurret *m_turret;
	bool m_isFinished;
	float m_allowedRange;
};

class FireFrisbeeCommand : public Command {
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


class EjectFrisbeeCommand : public Command {
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

class LoadAndFireCommand : public CommandGroup {
public:
	LoadAndFireCommand(
		BaseFrisbeeLoader *loader, 
		BaseFrisbeeAimer *aimer, 
		BaseFrisbeeTurret *turret, 
		BaseFrisbeeShooter *shooter);
	~LoadAndFireCommand();
};

#endif
