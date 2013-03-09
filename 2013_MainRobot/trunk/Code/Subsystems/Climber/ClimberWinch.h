#ifndef CLIMBER_WINCH_H
#define CLIMBER_WINCH_H
/**
 * \file ClimberWinch.h
 * 
 * \addtogroup subsystems
 * \{
 */

#include "WPILib.h"
#include "../BaseSubsystem.h"
#include"../../Misc/Tools.h"

/**
 * \brief Code to operate the motor to lift up the robot a rung once hooked on.
 */
namespace Winch{

class Base : public BaseSubsystem {
public:
	Base(const char *name);
	~Base();
	
	virtual void SetSpeed(double speed) = 0;
	virtual double GetSpeed() = 0;
};

class Simple: public Base {
public:
	Simple(SpeedController *motor);
	~Simple();
	
	void SetSpeed(double speed);
	double GetSpeed();
private:
	SpeedController *m_motor;
};

}

/**
 * \}
 */

#endif
