#ifndef FRISBEE_RAISER_H
#define FRISBEE_RAISER_H

#include "WPILib.h"
#include "../BaseSubsystem.h"
#include "../../Misc/Tools.h"

/**
 * A class to lower and raise the shooter so that it's easier to hit 
 * higher targets without being blocked.
 */
class BaseFrisbeeRaiser : public BaseSubsystem {
public:
	BaseFrisbeeRaiser(const char *name);
	virtual ~BaseFrisbeeRaiser();
	
	/**
	 * Returns true if the lift is at the very top.
	 */
	virtual bool IsAtTop() = 0;
	
	/**
	 * Returns true if the lift is at the very bottom.
	 */
	virtual bool IsAtBottom() = 0;
	
	/**
	 * Simply raises the elevator by some increment (unless it's at 
	 * top). You must call 'Raise' repeatedly in a loop to actually lift
	 * the elevator.
	 */
	virtual void Raise() = 0;
	
	/**
	 * Same thing as Raise(), but lowers the lift instead.
	 */
	virtual void Lower() = 0;
	
	/**
	 * Manually set the speed controller to a certain speed to raise the lift.
	 */
	virtual void Raise(float magnitude) = 0;
	
	/**
	 * Same as Raise(float) but lowers the lift.
	 */
	virtual void Lower(float magnitude) = 0;
	
	/**
	 * Forcibly stop the elevator.
	 */
	virtual void Stop() = 0;
};

/**
 * FrisbeeRaise class that uses limit switches at the top and bottom to prevent
 * the raiser from going to high or too low.
 */
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
