/**
 * \file FrisbeeTurret.h
 * 
 * \brief Allows the turret to move horizontally and vertically
 * 
 * This subsystem only takes into account one axis of control, such as horizontal or vertical.
 * We took into account only one axis because   	
 */


#ifndef FRISBEE_TURRET_H
#define FRISBEE_TURRET_H

#include "WPILib.h"
#include "../BaseSubsystem.h"
#include "FrisbeeAimer.h"

namespace FrisbeeTurret {

class Base : public BaseSubsystem {
public: 
	Base(const char *name);
	virtual ~Base();
	
	virtual void SetSpeed(float speed) = 0;
	virtual void Stop() = 0;
};

class Simple : public Base {
public:
	Simple(SpeedController *motor, const char *name);
	~Simple();
	
	void SetSpeed(float speed);
	void Stop();
	
private: 
	SpeedController *m_motor;
	double k_highSpeed;
	double k_mediumSpeed;
	double k_lowSpeed;
};

class Guarded : public Base {
public:
	Guarded(
		SpeedController *motor,  
		DigitalInput *switch1,
		DigitalInput *switch2,
		const char *name);
	~Guarded();
	
	void SetSpeed(float speed);
	void Stop();
	
private:
	SpeedController *m_motor; 
	DigitalInput *m_switch1;
	DigitalInput *m_switch2;
};

}


namespace TurretPosition {

enum Position {
	kError,
	kLeft,
	kCenter,
	kRight
};

class Base : public BaseSubsystem {
public:	
	Base(const char *name);
	~Base();
	
	virtual Position GetPosition() = 0;
	virtual float GetAngle() = 0;
	virtual void SetHome(float angle) = 0;
};

class EncoderAngle : public Base {
public:
	EncoderAngle(Encoder *encoder, float home, const char *name);
	~EncoderAngle();
	
	Position GetPosition();
	float GetAngle();
	void SetHome(float angle);
	
private:
	Encoder *m_encoder;
	float m_home;
};

class TwoLimitSwitches : public Base {
public:
	TwoLimitSwitches(DigitalInput *leftSwitch, DigitalInput *rightSwitch);
	~TwoLimitSwitches();
	
	Position GetPosition();
	float GetAngle();
	void SetHome(float angle);
	
private:
	DigitalInput *m_leftSwitch;
	DigitalInput *m_rightSwitch;
};

}



#endif
