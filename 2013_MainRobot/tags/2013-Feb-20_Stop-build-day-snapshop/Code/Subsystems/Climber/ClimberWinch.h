#ifndef CLIMBER_WINCH_H
#define CLIMBER_WINCH_H

#include "WPILib.h"
#include "../BaseSubsystem.h"
#include"../../Misc/Tools.h"

class BaseWinch : public BaseSubsystem {
public:
	BaseWinch(const char *name);
	~BaseWinch();
	
	virtual void SetSpeed(double speed) = 0;
	virtual double GetSpeed() = 0;
};

class SimpleWinch: public BaseWinch {
public:
	SimpleWinch(SpeedController *motor);
	~SimpleWinch();
	
	void SetSpeed(double speed);
	double GetSpeed();
private:
	SpeedController *m_motor;
};

#endif
