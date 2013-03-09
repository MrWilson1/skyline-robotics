#ifndef SHOOTER_COMMANDS_H
#define SHOOTER_COMMANDS_H

#include "WPILib.h"
#include "../Misc/Tools.h"
#include "BaseCommand.h"

#include "../Subsystems/Shooter/FrisbeeAimer.h"
#include "../Subsystems/Shooter/FrisbeeLoader.h"
#include "../Subsystems/Shooter/FrisbeeShooter.h"
#include "../Subsystems/Shooter/FrisbeeTurret.h"

#include "../Subsystems/Controllers/Axis.h"

/**
 * \brief Command for loading frisbees.
 */
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

/**
 * \brief Aims the turret at the given target
 * 
 * Protip: If you set this to the default turret command, using
 * 
 * 	   Tracking::ClosestOffset
 * 	   
 * ...as the desiredTarget param, the turret will automatically
 * lock on and track the target it's pointing at.
 * 
 * You can manually adjust the target to point at a different target,
 * or issue a command to point it at the appropriate target and let
 * this take over once that command is finished.
 */
class AimTurretCommand : public Command {
public:
	AimTurretCommand(
			BaseFrisbeeAimer *aimer, 
			BaseAxisFrisbeeTurret *horizontalTurret, 
			BaseAxisFrisbeeTurret *verticalTurret,
			Tracking::TargetType desiredTarget, 
			double allowedRange);
	AimTurretCommand(
			BaseFrisbeeAimer *aimer, 
			BaseAxisFrisbeeTurret *horizontalTurret,
			BaseAxisFrisbeeTurret *verticalTurret, 
			Tracking::TargetType desiredTarget, 
			double allowedRange,
			double lowSpeed,
			double mediumSpeed,
			double highSpeed);
			
	~AimTurretCommand();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
	
private:
	BaseFrisbeeAimer *m_aimer;
	BaseAxisFrisbeeTurret *m_horizontalTurret;
	BaseAxisFrisbeeTurret *m_verticalTurret;
	bool m_isFinished;
	Tracking::TargetType m_desiredTarget;
	double m_allowedRange;
	double k_lowSpeed;
	double k_mediumSpeed;
	double k_highSpeed;
};

/**
 * \brief Fires the frisbee at a default distance or a set distance.
 */
class FireFrisbeeCommand : public Command {
public:
	FireFrisbeeCommand(BaseFrisbeeShooter *shooter);
	~FireFrisbeeCommand();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
		
private:
	BaseFrisbeeShooter *m_shooter;
};


/**
 * \brief Loads a frisbee, and just fires it without aiming
 */
class LoadAndFireFrisbeeCommand : public CommandGroup {
public:
	LoadAndFireFrisbeeCommand(
		BaseFrisbeeLoader *loader,
		BaseFrisbeeShooter *shooter);
	~LoadAndFireFrisbeeCommand();
};

/**
 * \brief Loads a frisbee, aims the turret, and fires it.
 */
class LoadAimAndFireCommand : public CommandGroup {
public:
	LoadAimAndFireCommand(
		BaseFrisbeeLoader *loader, 
		BaseFrisbeeAimer *aimer, 
		BaseAxisFrisbeeTurret *horizontalTurret, 
		BaseAxisFrisbeeTurret *verticalTurret,
		BaseFrisbeeShooter *shooter);
	~LoadAimAndFireCommand();
};

/**
 * \brief Manually adjusts where the turret is pointing by plugging in raw axis speeds
 */
class AdjustTurretCommand : public SimpleCommand {
public:
	AdjustTurretCommand(
		BaseAxisFrisbeeTurret *horizontalTurret,
		BaseAxisFrisbeeTurret *verticalTurret,
		double rotateSpeed,
		double verticalSpeed, 
		double allowedRange);
	~AdjustTurretCommand();
	void Execute();
	
private:
	BaseAxisFrisbeeTurret *m_horizontalTurret;
	BaseAxisFrisbeeTurret *m_verticalTurret;
	double m_rotateSpeed;
	double m_verticalSpeed;
	bool m_isFinished;
};


class ManuallyControlTurretCommand : public SimpleCommand {
public:
	ManuallyControlTurretCommand(BaseAxisFrisbeeTurret *turretAxis, Axis *inputAxis, const char *name);
	~ManuallyControlTurretCommand();
	void Execute();
	
private:
	BaseAxisFrisbeeTurret *m_turret;
	Axis *m_axis;
};

#endif
