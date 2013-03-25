#ifndef SHOOTER_H
#define SHOOTER_H

#include "WPILib.h"

#include "../BaseSubsystem.h"
#include "../../Misc/Tools.h"

namespace FrisbeeShooter {

/**
 * Base class for a Frisbee shooter.
 * 
 * Can either shoot Frisbees, shoot them at a certain speed
 * (implementation-specific), or lightly eject them so they
 * dribble out of the robot.
 * 
 * Note: this interface may change substantially over time.
 */
class Base : public BaseSubsystem {
public:
	Base(const char *name);
	virtual ~Base();
	
	/**
	 * Shoots the frisbee at a preset speed.
	 */
	virtual void ShootFrisbee() = 0;
	
	/**
	 * Shoots the frisbee at a specified speed.
	 */
	virtual void ShootFrisbee(float inchesPerSecond) = 0;
	virtual void StopFrisbee() = 0;
	virtual void SetFrisbeeSpeed(double speed) = 0;
};

class SimpleBelt : public Base {
public:
	enum Direction {
		Positive = 1,
		Negative = -1,
	};
	
	SimpleBelt(SpeedController *motor);
	~SimpleBelt();
	
	void ShootFrisbee();
	void ShootFrisbee(float inchesPerSEcond);
	void StopFrisbee();
	void SetFrisbeeSpeed(double speed);

private:
	SpeedController *m_motor;
};

class ThreeWheel : public Base {
public:
	ThreeWheel(SpeedController *frontMotor, SpeedController *middleMotor, SpeedController *lastMotor);
	~ThreeWheel();
	
	void ShootFrisbee();
	void ShootFrisbee(float distanceInInches);
	void StopFrisbee();
	void SetFrisbeeSpeed(double speed);
	
private:
	SpeedController *m_frontMotor;
	SpeedController *m_middleMotor;
	SpeedController *m_lastMotor;
};


class ThreeWheelSteady : public Base {
public:
	ThreeWheelSteady(
			SpeedController *frontMotor, 
			SpeedController *middleMotor, 
			SpeedController *lastMotor,
			Encoder *encoder,
			float maxEncoderPulses,
			float maxEncoderSpeed);
	~ThreeWheelSteady();
	
	void ShootFrisbee();
	void ShootFrisbee(float distanceInInches);
	void StopFrisbee();
	void SetFrisbeeSpeed(double speed);
	
private:
	SpeedController *m_frontMotor;
	SpeedController *m_middleMotor;
	SpeedController *m_lastMotor;
	Encoder *m_encoder;
	const float m_maxEncoderPulses;
	const float m_maxEncoderSpeed;
	float m_targetSpeed;
	float m_outputSpeed;
};


}

#endif
