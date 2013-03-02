#ifndef CLIMBER_JOINTS_H
#define CLIMBER_JOINTS_H

#include "WPILib.h"
#include "../BaseSubsystem.h"
#include"../../Misc/Tools.h"

class BaseJoint : public BaseSubsystem {
public:
	BaseJoint(const char *name);
	~BaseJoint();
	
	virtual void SetSpeed(double speed) = 0;
	virtual double GetSpeed() = 0;
};

class BaseSmartJoint : public BaseJoint {
public:
	BaseSmartJoint(const char *name);
	~BaseSmartJoint();
	
	virtual void SetAngle(double angle) = 0;
	virtual double GetAngle() = 0;
};

/**
 * \brief Joint for climber that uses winch and motor.
 */
class BasicWinchClimberJoint : public BaseJoint {
public:
	BasicWinchClimberJoint(SpeedController *motor);
	~BasicWinchClimberJoint();
	
	void SetSpeed(double speed);
	double GetSpeed();
private:
	SpeedController *m_motor;
};

/**
 * \brief Joint for climber that uses a PID controller, encoders and a winch.
 */
class PidWinchClimberJoint : public BaseSmartJoint {
public:
	PidWinchClimberJoint(SpeedController *motor, Encoder *encoder);
	~PidWinchClimberJoint();
	
	/**
	 * \brief Sets angle of joint in degrees.
	 */
	void SetAngle(float degrees);
	/**
	 * \brief Gets angle of joint in degrees.
	 */
	double GetAngle();
	void SetSpeed(double speed);
	double GetSpeed();
	
private:
	SpeedController *m_motor;
	Encoder *m_encoder;	
	PIDController *m_pid;
};


#endif
