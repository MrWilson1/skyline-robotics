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
 * \brief Contains commands related to loading, aiming, and shooting
 * the frisbee.
 */
namespace ShooterCommand {
/**
 * \brief Loads the frisbee
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
 * \brief Fires the frisbee at a given speed.
 */
class FireFrisbee : public Command {
public:
	FireFrisbee(FrisbeeShooter::Base *shooter);
	FireFrisbee(FrisbeeShooter::Base *shooter, float speed);
	~FireFrisbee();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
		
private:
	FrisbeeShooter::Base *m_shooter;
	float m_speed;
};

class FireFrisbeeWithAdjustableSpeed : public SimpleCommand {
public:
	FireFrisbeeWithAdjustableSpeed(FrisbeeShooter::Base *shooter, BaseAxis *speedAxis);
	~FireFrisbeeWithAdjustableSpeed();
	void Execute();
	void End();
	void Interrupted();
	
private:
	FrisbeeShooter::Base *m_shooter;
	BaseAxis *m_speedAxis;
};

class SmartDashboardFireFrisbee : public SimpleCommand {
public:
	SmartDashboardFireFrisbee(FrisbeeShooter::Base *shooter);
	SmartDashboardFireFrisbee(FrisbeeShooter::Base *shooter, double conversionFactor);
	~SmartDashboardFireFrisbee();
	void Execute();
private:
	FrisbeeShooter::Base *m_shooter;
	double m_conversionFactor;
};


/**
 * \brief Loads a frisbee, and fires it.
 */
class LoadAndFire : public CommandGroup {
public:
	LoadAndFire(
		FrisbeeLoader::Base *loader, 
		FrisbeeShooter::Base *shooter);
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

class AdjustTurretAngle : public SimpleCommand {
public:
	AdjustTurretAngle(
			FrisbeeTurret::Base *turret,
			TurretPosition::Base *position,
			float angle);
	~AdjustTurretAngle();
	
	void Execute();
	bool IsFinished();
	
private:
	FrisbeeTurret::Base *m_turret;
	TurretPosition::Base *m_position;
	float m_angle;
};


/**
 * Command to manually move the horizontal and vertical turrets using Axes.
 */
class ManuallyControlTurret : public SimpleCommand {
public:
	ManuallyControlTurret(
			FrisbeeTurret::Base *turretAxis, 
			BaseAxis *inputAxis, 
			const char *name);
	~ManuallyControlTurret();
	void Execute();
	
private:
	FrisbeeTurret::Base *m_turret;
	BaseAxis *m_axis;
};


class MoveTurretHome : public SimpleCommand {
public:
	MoveTurretHome(
			FrisbeeTurret::Base *turretAxis,
			TurretPosition::Base *position,
			const char*name);
	~MoveTurretHome();
	void Execute();
	bool IsFinished();
	
private:
	FrisbeeTurret::Base *m_turret;
	TurretPosition::Base *m_position;
};


class SetTurretHome : public SimpleCommand {
public:
	SetTurretHome(
			TurretPosition::Base *position,
			const char *name);
	~SetTurretHome();
	void Execute();
	
private:
	TurretPosition::Base *m_position;
};


}

/**
 * \}
 */
#endif
