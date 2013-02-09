#ifndef CLIMBER_JOINTS_H
#define CLIMBER_JOINTS_H

#include "WPILib.h"
#include "../BaseSubsystem.h"

class BaseClimberJoint : public BaseSubsystem {
public:
	BaseClimberJoint(const char *name);
	virtual ~BaseClimberJoint();
	
	virtual void SetAngle(float degrees) = 0;
	virtual float GetAngle() = 0;
};

class MotorClimberJoint : public BaseSubsystem {
public:
	MotorClimberJoint(SpeedController *motor, Encoder *encoder);
	~MotorClimberJoint();
	
	void SetAngle(float degrees);
	float GetAngle();
	
private:
	SpeedController *m_motor;
	Encoder *m_encoder;
};

class WinchClimberJoint : public BaseSubsystem {
public:
	WinchClimberJoint(SpeedController *motor, Encoder *encoder);
	~WinchClimberJoint();
	
	void SetAngle(float degrees);
	float GetAngle();
	
private:
	SpeedController *m_motor;
	Encoder *m_encoder;	
};


#endif
