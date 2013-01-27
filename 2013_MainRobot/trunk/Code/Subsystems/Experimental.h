#ifndef EXPERIMENTAL_H
#define EXPERIMENTAL_H

#include "WPILib.h"
#include "BaseSubsystem.h"

class TestMotor : public BaseSubsystem {
public:
	TestMotor(SpeedController *motor, const char *name);
	~TestMotor();
	
	void SetSpeed(float speed);
	void MaxSpeed();
	
public:
	SpeedController *m_motor;
};

class TestEncoder : public BaseSubsystem {
public:
	TestEncoder(Encoder *encoder, const char *name);
	~TestEncoder();
	
	void SetDistancePerPulse(double input);
	void Run();
	
private:
	Encoder *m_encoder;
};


#endif
