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



}

/**
 * \}
 */
#endif
