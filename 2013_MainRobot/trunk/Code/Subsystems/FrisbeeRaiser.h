#ifndef FRISBEE_RAISER_H
#define FRISBEE_RAISER_H

#include "WPILib.h"
#include "BaseSubsystem.h"
#include "../Misc/Tools.h"

/**
 * A class to lower and raise the shooter so that it's easier to hit 
 * higher targets without being blocked.
 * 
 * Methods:
 * - IsAtTop()
 * 		returns true if the lift is at the very top
 * - IsAtBottom()
 * 		returns true if the lift is at the very bottom
 * - Raise()
 * 		simply raises the elevator by some increment (unless it's at 
 * 		top). You must call 'Raise' repeatedly in a loop to actually lift
 * 		the elevator.
 * - Lower()
 * 		the same thing, except to lower the lift
 * - Raise(float magnitude)
 * 		manually set the speed controller to go up by some speed
 * - Lower(float magnitude)
 * 		Same thing, except to lower.
 * - Stop()
 * 		Forcibly stop the elevator
 */
class BaseFrisbeeRaiser : public BaseSubsystem {
public:
	BaseFrisbeeRaiser(const char *name);
	virtual ~BaseFrisbeeRaiser();
	
	virtual bool IsAtTop() = 0;
	virtual bool IsAtBottom() = 0;
	virtual void Raise() = 0;
	virtual void Lower() = 0;
	virtual void Raise(float magnitude) = 0;
	virtual void Lower(float magnitude) = 0;
	virtual void Stop() = 0;
};

class GuardedFrisbeeRaiser : public BaseFrisbeeRaiser {
public:
	GuardedFrisbeeRaiser(SpeedController *raiserMotor, DigitalInput *topLimitSwitch, DigitalInput *bottomLimitSwitch, double motorSpeed);
	~GuardedFrisbeeRaiser();
	
	bool IsAtTop();
	bool IsAtBottom();
	void Raise();
	void Lower();
	void Raise(float magnitude);
	void Lower(float magnitude);
	void Stop();
private:
	double m_motorSpeed; // may change later when we test it
	SpeedController *m_raiserMotor;
	DigitalInput *m_topLimitSwitch;
	DigitalInput *m_bottomLimitSwitch;
};

#endif
