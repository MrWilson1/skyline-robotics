#ifndef SHOOTER_H
#define SHOOTER_H

#include "WPILib.h"

#include "BaseSubsystem.h"
#include "../Misc/Tools.h"

/**
 * Base class for a Frisbee shooter.
 * 
 * Can either shoot Frisbees, shoot them at a certain speed
 * (implementation-specific), or lightly eject them so they
 * dribble out of the robot.
 * 
 * Note: this interface may change substantially over time.
 */
class BaseFrisbeeShooter : public BaseSubsystem {
public:
	BaseFrisbeeShooter(const char *name);
	virtual ~BaseFrisbeeShooter();
	
	virtual void ShootFrisbee() = 0;
	virtual void ShootFrisbee(float inchesPerSecond) = 0;
	virtual void EjectFrisbee() = 0;
};

class SimpleBeltShooter : public BaseFrisbeeShooter {
public:
	enum Direction {
		Positive = 1,
		Negative = -1,
	};
	
	SimpleBeltShooter(SpeedController *motor, Direction forwardDirection);
	~SimpleBeltShooter();
	
	void ShootFrisbee();
	void ShootFrisbee(float distanceInInches);
	void EjectFrisbee();
	
	void SetMaxDistance(float inches);
	float GetMaxDistance();

private:
	SpeedController *m_motor;
	Direction m_forwardDirection;
	float m_maxDistance;
};

class ThreeWheelShooter : public BaseFrisbeeShooter {
public:
	ThreeWheelShooter(SpeedController *frontMotor, SpeedController *middleMotor, SpeedController *lastMotor);
	~ThreeWheelShooter();
	
	void ShootFrisbee();
	void ShootFrisbee(float distanceInInches);
	void EjectFrisbee();
	void SetMaxDistance(float inches);
	float GetMaxdistance();
	
private:
	SpeedController *m_frontMotor;
	SpeedController *m_middleMotor;
	SpeedController *m_lastMotor;
	float m_maxDistance;
};

#endif
