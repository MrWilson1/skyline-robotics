#ifndef CLIMBER_JOINTS_H
#define CLIMBER_JOINTS_H

#include "WPILib.h"
#include "../BaseSubsystem.h"

/**
 * \brief Base class for all climber joints.
 */
class BaseClimberJoint : public BaseSubsystem {
public:
	BaseClimberJoint(const char *name);
	virtual ~BaseClimberJoint();
	
	/**
	 * \brief Sets angle of joint in degrees.
	 */
	virtual void SetAngle(float degrees) = 0;
	/**
	 * \brief Gets angle of joint in degrees.
	 */
	virtual float GetAngle() = 0;
};

/**
 * \brief Joint for climber that uses motors.
 */
class MotorClimberJoint : public BaseClimberJoint {
public:
	MotorClimberJoint(SpeedController *motor, Encoder *encoder);
	~MotorClimberJoint();
	
	/**
	 * \brief Sets angle of joint in degrees.
	 */
	void SetAngle(float degrees);
	/**
	 * \brief Gets angle of joint in degrees.
	 */
	float GetAngle();
	
private:
	SpeedController *m_motor;
	Encoder *m_encoder;
	PIDController *m_pid;
};

/**
 * \brief Joint for climber that uses a winch.
 */
class WinchClimberJoint : public BaseClimberJoint {
public:
	WinchClimberJoint(SpeedController *motor, Encoder *encoder);
	~WinchClimberJoint();
	
	/**
	 * \brief Sets angle of joint in degrees.
	 */
	void SetAngle(float degrees);
	/**
	 * \brief Gets angle of joint in degrees.
	 */
	float GetAngle();
private:
	SpeedController *m_motor;
	Encoder *m_encoder;	
	PIDController *m_pid;
};


#endif
