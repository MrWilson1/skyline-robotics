#ifndef SHOOTER_H
#define SHOOTER_H

#include "WPILib.h"

/**
 * Base class for a Frisbee shooter.
 * 
 * Can either shoot Frisbees, shoot them at a certain speed
 * (implementation-specific), or lightly eject them so they
 * dribble out of the robot.
 * 
 * Note: this interface may change substantially over time.
 */
class BaseFrisbeeShooter : public Subsystem {
public:
	BaseFrisbeeShooter(const char *name);
	virtual ~BaseFrisbeeShooter();
	
	virtual void ShootFrisbee() = 0;
	virtual void ShootFrisbee(float inchesPerSecond) = 0;
	virtual void EjectFrisbee() = 0;
};

#endif
