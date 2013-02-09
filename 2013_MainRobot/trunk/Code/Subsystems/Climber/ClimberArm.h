#ifndef CLIMBER_ARM_H
#define CLIMBER_ARM_H

#include <cmath>
#include "WPILib.h"
#include "../BaseSubsystem.h"

class BaseClimberArm : public BaseSubsystem {
public:
	BaseClimberArm();
	virtual ~BaseClimberArm();
	
	virtual void SetPolar(float degrees, float magnitude) = 0;
	virtual void SetCartesian(float x, float y) = 0;
	virtual float GetAngle() = 0;
	virtual float GetMagnitude() = 0;
	virtual float GetX() = 0;
	virtual float GetY() = 0;
};

/**
class BungeeWinchClimberArm : public BaseClimberArm {
public:
	
};*/

#endif
