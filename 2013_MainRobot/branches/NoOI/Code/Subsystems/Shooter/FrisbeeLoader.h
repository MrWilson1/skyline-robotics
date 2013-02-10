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
	
	virtual int GetNumberOfFrisbeesLoaded() = 0;
	virtual void UnloadFrisbee() = 0;
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
	
	int GetNumberOfFrisbeesLoaded();
	void UnloadFrisbee();
	void PrepareFrisbee();
	bool IsFrisbeePrepared();
	void LoadFrisbee();
};

#endif
