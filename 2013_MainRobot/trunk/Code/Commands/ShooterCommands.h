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

namespace ShooterCommand {
/**
 * \brief Command for loading frisbees.
 */
class LoadFrisbee : public Command {
public:
	LoadFrisbee(FrisbeeLoader::Base *loader);
	~LoadFrisbee();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
	
private:
	FrisbeeLoader::Base *m_loader;
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
class AimTurret : public Command {
public:
	AimTurret(
			FrisbeeAimer::Base *aimer, 
			FrisbeeTurret::Base *horizontalTurret, 
			FrisbeeTurret::Base *verticalTurret,
			Tracking::TargetType desiredTarget,
			double allowedRange);			
	~AimTurret();

	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
	
private:
	FrisbeeAimer::Base *m_aimer;
	FrisbeeTurret::Base *m_horizontalTurret;
	FrisbeeTurret::Base *m_verticalTurret;
	bool m_isFinished;
	Tracking::TargetType m_desiredTarget;
	double m_allowedRange;
};


/**
 * \brief Fires the frisbee at a default distance or a set distance.
 */
class FireFrisbee : public Command {
public:
	FireFrisbee(BaseFrisbeeShooter *shooter);
	~FireFrisbee();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
		
private:
	BaseFrisbeeShooter *m_shooter;
};


/**
 * \brief Loads a frisbee, and fires it.
 */
class LoadAndFire : public CommandGroup {
public:
	LoadAndFire(
		FrisbeeLoader::Base *loader, 
		BaseFrisbeeShooter *shooter);
	~LoadAndFire();
};


/**
 * \brief Manually adjusts where the turret is pointing by plugging in raw axis speeds
 */
class AdjustTurret : public SimpleCommand {
public:
	AdjustTurret(
		FrisbeeTurret::Base *horizontalTurret,
		FrisbeeTurret::Base *verticalTurret,
		double rotateSpeed,
		double verticalSpeed, 
		double allowedRange);
	~AdjustTurret();
	void Execute();
	
private:
	FrisbeeTurret::Base *m_horizontalTurret;
	FrisbeeTurret::Base *m_verticalTurret;
	double m_rotateSpeed;
	double m_verticalSpeed;
	bool m_isFinished;
};


/**
 * Command to manually move the horizontal and vertical turrets using Axes.
 */
class ManuallyControlTurret : public SimpleCommand {
public:
	ManuallyControlTurret(
			FrisbeeTurret::Base *turretAxis, 
			Axis *inputAxis, 
			const char *name);
	~ManuallyControlTurret();
	void Execute();
	
private:
	FrisbeeTurret::Base *m_turret;
	Axis *m_axis;
};

}

/**
 * \}
 */
#endif
