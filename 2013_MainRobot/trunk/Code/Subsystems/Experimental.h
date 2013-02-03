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
	
	void ReportAll();
	
	void Start();
	void Stop();
	void Reset();
	void SetMaxPeriod(double maxPeriod);
	void SetMinRate(double minRate);
	void SetDistancePerPulse(double input);
	void SetReverseDirection(bool reverseDirection);
	void SetPIDSourceParameter(Encoder::PIDSourceParameter pidSource);
	
	double GetDistancePerPulse();
	
private:
	void Log(const char *key, int value);
	void Log(const char *key, double value);
	void LogBool(const char *key, bool value);
	Encoder *m_encoder;
	double m_distancePerPulse;
};


#endif
