#ifndef SHOOTER_COMMANDS_H
#define SHOOTER_COMMANDS_H
/**
 * \file ShooterCommands.h
 * 
 * \addtogroup commands
 * \{
 */

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
 * \brief Loads a frisbee, aims the turret, and fires it.
 */
class LoadAndFireCommand : public CommandGroup {
public:
	LoadAndFireCommand(
		BaseFrisbeeLoader *loader, 
		BaseFrisbeeAimer *aimer, 
		BaseAxisFrisbeeTurret *horizontalTurret, 
		BaseAxisFrisbeeTurret *verticalTurret,
		BaseFrisbeeShooter *shooter);
	~LoadAndFireCommand();
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


/**
 * Command to manually move the horizontal and vertical turrets using Axes.
 */
class ManuallyControlTurretCommand : public SimpleCommand {
public:
	ManuallyControlTurretCommand(BaseAxisFrisbeeTurret *horizontalTurret, BaseAxisFrisbeeTurret *verticalTurret, Axis *verticalAxis, Axis *rotateAxis);
	~ManuallyControlTurretCommand();
	void Execute();
	
private:
	BaseAxisFrisbeeTurret *m_horizontalTurret;
	BaseAxisFrisbeeTurret *m_verticalTurret;
	Axis *m_verticalAxis;
	Axis *m_rotateAxis;
};

/**
 * \}
 */
#endif
