#ifndef FRISBEE_LOADER_H
#define FRISBEE_LOADER_H

#include "WPILib.h"

#include "../BaseSubsystem.h"

namespace FrisbeeLoader {
/**
 * A base class to load frisbees into the shooter.
 */
class Base : public BaseSubsystem {
public:
	Base(const char *name);
	virtual ~Base();
	
	/**
	 * Moves a frisbee in position so it's almost ready to shoot.
	 */
	virtual void PrepareFrisbee() = 0;
	
	/**
	 * Returns true if the loader is ready to inject a frisbee
	 * into the shooter.
	 */
	virtual bool IsFrisbeePrepared() = 0;
	
	/**
	 * Loads a frisbee into the shooter.
	 */
	virtual void LoadFrisbee() = 0;
};

/**
 * Doesn't actually do anything -- a placeholder until we can
 * actually code this.
 */
class Placeholder : public Base {
public:
	Placeholder();
	~Placeholder();
	
	void PrepareFrisbee();
	bool IsFrisbeePrepared();
	void LoadFrisbee();
};

/**
 * FrisbeeLoader class that uses a piston to load frisbees.
 */
class Piston : public Base {
public:
	Piston(Solenoid *piston);
	~Piston();
	
	void PrepareFrisbee();
	bool IsFrisbeePrepared();
	void LoadFrisbee();
private:
	Solenoid *m_piston;
};

/**
 * FrisbeeLoader class that uses a servo motor to load frisbees.
 */
class ServoLoader : public Base {
public:
	ServoLoader(Servo *servo);
	~ServoLoader();
	
	void PrepareFrisbee();
	bool IsFrisbeePrepared();
	void LoadFrisbee();
	
private:
	Servo *m_servo;
};
}



#endif
