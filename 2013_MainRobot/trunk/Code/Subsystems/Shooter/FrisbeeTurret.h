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
	Simple(SpeedController *motor);
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
	Guarded(SpeedController *motor,  
							 DigitalInput *switch1,
							 DigitalInput *switch2);
	~Guarded();
	
	void SetSpeed(float speed);
	void Stop();
	
private:
	SpeedController *m_motor; 
	DigitalInput *m_switch1;
	DigitalInput *m_switch2;
};

}


class BaseTurretPositionDetector : public BaseSubsystem {
public:
	enum Position {
		kLeft,
		kCenter,
		kRight
	};
	BaseTurretPositionDetector(const char *name);
	~BaseTurretPositionDetector();
	
	virtual Position GetPosition() = 0;
};



#endif
