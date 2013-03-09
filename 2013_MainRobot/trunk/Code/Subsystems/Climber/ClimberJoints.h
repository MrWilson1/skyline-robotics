#ifndef CLIMBER_JOINTS_H
#define CLIMBER_JOINTS_H
/**
 * \file ClimberJoints.h
 * 
 * \addtogroup subsystems
 * \{
 */

#include "WPILib.h"
#include "../BaseSubsystem.h"
#include"../../Misc/Tools.h"

namespace Joint {

class Base : public BaseSubsystem {
public:
	Base(const char *name);
	~Base();
	
	virtual void SetSpeed(double speed) = 0;
	virtual double GetSpeed() = 0;
};

class BaseSmart : public Base {
public:
	BaseSmart(const char *name);
	~BaseSmart();
	
	virtual void SetAngle(double angle) = 0;
	virtual double GetAngle() = 0;
};

/**
 * \brief Joint for climber that uses winch and motor.
 */
class BasicWinch : public Base {
public:
	BasicWinch(SpeedController *motor);
	~BasicWinch();
	
	void SetSpeed(double speed);
	double GetSpeed();
private:
	SpeedController *m_motor;
};

/**
 * \brief Joint for climber that uses a PID controller, encoders and a winch.
 */
class PidWinch : public BaseSmart {
public:
	PidWinch(SpeedController *motor, Encoder *encoder);
	~PidWinch();
	
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

}

/**
 * \}
 */


#endif
