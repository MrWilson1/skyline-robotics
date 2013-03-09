#ifndef FRISBEE_TURRET_H
#define FRISBEE_TURRET_H

#include "WPILib.h"

#include "../BaseSubsystem.h"
#include "FrisbeeAimer.h"


class BaseAxisFrisbeeTurret : public BaseSubsystem {
public: 
	BaseAxisFrisbeeTurret(const char *name);
	virtual ~BaseAxisFrisbeeTurret();
	
	virtual void SetMotor(float speed) = 0;
	virtual void Stop() = 0;
	virtual bool ShouldTurretStop() = 0;
	virtual void TurnGivenOffset(Tracking::Offset offset, double direction, double upper, double middle, double lower) = 0;
};

class SimpleAxisFrisbeeTurret : public BaseAxisFrisbeeTurret {
public:
	SimpleAxisFrisbeeTurret(SpeedController *motor);
	~SimpleAxisFrisbeeTurret();
	
	void SetMotor(float speed);
	void Stop();
	bool ShouldTurretStop();
	void TurnGivenOffset(Tracking::Offset offset, double direction, double upper, double middle, double lower);
	
private: 
	SpeedController *m_motor;
	double k_highSpeed;
	double k_mediumSpeed;
	double k_lowSpeed;
};

class GuardedAxisFrisbeeTurret : public BaseAxisFrisbeeTurret {
public:
	GuardedAxisFrisbeeTurret(SpeedController *motor,  
							 DigitalInput *switch1,
							 DigitalInput *switch2);
	~GuardedAxisFrisbeeTurret();
	
	void SetMotor(float speed);
	void Stop();
	bool ShouldTurretStop();
	void TurnGivenOffset(Tracking::Offset offset, double direction, double upper, double middle, double lower);
	
private:
	SpeedController *m_motor; 
	DigitalInput *m_switch1;
	DigitalInput *m_switch2;
	double k_highSpeed;
	double k_mediumSpeed;
	double k_lowSpeed;
};



#endif
