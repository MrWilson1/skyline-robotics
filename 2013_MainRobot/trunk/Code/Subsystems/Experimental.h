#ifndef EXPERIMENTAL_H
#define EXPERIMENTAL_H

#include "WPILib.h"
#include "BaseSubsystem.h"

class TestMotor : public BaseSubsystem {
public:
	TestMotor(SpeedController *motor);
	~TestMotor();
	
	void SetSpeed(float speed);
	void MaxSpeed();
	
public:
	SpeedController *m_motor;
};


#endif
