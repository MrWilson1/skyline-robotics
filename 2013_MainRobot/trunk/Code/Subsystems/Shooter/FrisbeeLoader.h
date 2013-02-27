#ifndef FRISBEE_LOADER_H
#define FRISBEE_LOADER_H

#include "WPILib.h"

#include "../BaseSubsystem.h"

/**
 * A base class to load frisbees into the shooter.
 * 
 * Methods
 * - GetNumberOfFrisbeesLoaded()
 * 		Does the obvious
 * - UnloadFrisbees()
 * 		Sort of gently spits the Frisbees out (doesn't shoot)
 * 		in case we accidentally loaded a frisbee
 * - PrepareFrisbee()
 * 		Moves a frisbee in position so it's almost ready to shoot
 * - IsFrisbeePrepared()
 * 		Returns true if the loader is ready to inject a frisbee
 * 		into the shooter
 * - LoadFrisbee()
 * 		Loads a frisbee into the shooter
 */
class BaseFrisbeeLoader : public BaseSubsystem {
public:
	BaseFrisbeeLoader(const char *name);
	virtual ~BaseFrisbeeLoader();
	
	virtual void PrepareFrisbee() = 0;
	virtual bool IsFrisbeePrepared() = 0;
	virtual void LoadFrisbee() = 0;
};

/**
 * Doesn't actually do anything -- a placeholder until we can
 * actually code this.
 */
class PlaceholderFrisbeeLoader : public BaseFrisbeeLoader {
public:
	PlaceholderFrisbeeLoader();
	~PlaceholderFrisbeeLoader();
	
	void PrepareFrisbee();
	bool IsFrisbeePrepared();
	void LoadFrisbee();
};

class PistonFrisbeeLoader : public BaseFrisbeeLoader {
public:
	PistonFrisbeeLoader(Solenoid *piston);
	~PistonFrisbeeLoader();
	
	void PrepareFrisbee();
	bool IsFrisbeePrepared();
	void LoadFrisbee();
private:
	Solenoid *m_piston;
};

class ServoFrisbeeLoader : public BaseFrisbeeLoader {
public:
	ServoFrisbeeLoader(Servo *servo);
	~ServoFrisbeeLoader();
	
	void PrepareFrisbee();
	bool IsFrisbeePrepared();
	void LoadFrisbee();
	
private:
	Servo *m_servo;
};



#endif
