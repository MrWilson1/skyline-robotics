#ifndef FRISBEE_PICKUP_H
#define FRISBEE_PICKUP_H

#include "WPILib.h"
#include "BaseSubsystem.h"

/**
 * Picks up the frisbees from the ground
 * 
 * - PreparePickup()
 * 		Prepares the pickup (for example, if it's an arm+hopper based
 * 		scheme, lowers it). If the pickup doesn't need to do anything
 * 		to be ready, just define the function and leave it blank. 
 * - RetractPickup()
 * 		Stores the pickup away, if it extends beyond the frame of the
 * 		robot. If the pickup doesn't need to be stored away, just 
 * 		define the function and leave it blank
 * - IsPickupPrepared()
 * 		Returns true if the pickup is ready to run
 * - IsPickupReadyToLoad()
 * 		Returns true if the pickup is ready to pass the frisbee to the
 * 		loader
 * - IsPickupRetracted()
 * 		Returns true if the pickup is retracted safely inside the
 * 		robot.
 * - RunPickup()
 * 		Picks up a frisbee
 * - RunPickup(float speed)
 * 		Picks up a frisbee at a given speed
 * - PassFrisbeesToLoader()
 * 		Passes the frisbee to the loader
 * - RejectFrisbees()
 * 		If we accidentally picked up the frisbee, reject it and
 * 		spit it out
 */
class BaseFrisbeePickup : public BaseSubsystem {
public:
	BaseFrisbeePickup(const char *name);
	~BaseFrisbeePickup();
	
	virtual void PreparePickup() = 0;
	virtual void RetractPickup() = 0;
	virtual bool IsPickupPrepared() = 0;
	virtual bool IsPickupReadyToLoad() = 0;
	virtual bool IsPickupRetracted() = 0;
	
	virtual void RunPickup() = 0;
	virtual void RunPickup(float speed) = 0;
	
	virtual void PassFrisbeesToLoader() = 0;
	virtual void RejectFrisbees() = 0;
};

#endif
