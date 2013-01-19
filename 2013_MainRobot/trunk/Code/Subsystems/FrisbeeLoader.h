#ifndef FRISBEE_LOADER_H
#define FRISBEE_LOADER_H

#include "WPILib.h"

#include "BaseSubsystem.h"

/**
 * A base class to load frisbees into the shooter.
 * 
 * Note: this interface may change substantially over time.
 */
class BaseFrisbeeLoader : public BaseSubsystem {
	BaseFrisbeeLoader(const char *name);
	virtual ~BaseFrisbeeLoader();
	
	virtual int GetNumberOfFrisbeesLoaded() = 0;
	
	// If possible, rejects a Frisbee and spits it out
	// without shooting
	virtual void UnloadFrisbee() = 0;
	
	// Prepare a Frisbee so it's almost ready to shoot
	virtual void PrepareFrisbee() = 0;
	
	virtual void IsFrisbeePrepared() = 0;
	
	// Takes the final step and loads a Frisbee into the
	// shooter
	virtual void LoadFrisbee() = 0;
	
};

#endif
