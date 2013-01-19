#ifndef FRISBEE_AIMER_H
#define FRISBEE_AIMER_H

#include "WPILib.h"

#include "BaseSubsystem.h"

/**
 * The 4 different types of targets the robot can aim for
 */
enum TargetType {
	Low,
	Medium,
	High,
	Pyramid
};

/**
 * The offset in degrees needed to turn the shooter to point at the
 * target. If the shooter needs to move 3 degrees up, 5 degrees to
 * the left, the value returned would be {-5, 3}
 */
struct Offset {
	float XOffset;
	float YOffset;
};

/**
 * A struct representing a single target.
 */
struct Target {
	TargetType Target;
	Offset ShooterOffset;
	float DistanceInInches;
};

/**
 * A base class to find the targets to aim for.
 * 
 * Note: this interface may change substantially over time.
 */
class BaseFrisbeeAimer : public BaseSubsystem {
public:
	BaseFrisbeeAimer(const char *name);
	virtual ~BaseFrisbeeAimer();

	// Selects the target to aim for
	virtual bool SetTarget(TargetType target) = 0;
	
	// Is the target selected visible?
	virtual bool IsTargetVisible() = 0;
	
	// Return all targets in sight
	virtual std::vector<Target> GetAllTargets() = 0;
	
	// Return the offset needed to turn the shooter to point at
	// the target
	virtual Offset GetOffset() = 0;
	
	// Gets the distance from the robot to the target in inches.
	virtual float GetDistanceFromTargetInches() = 0;
	
	// Gets the type of target the sensor is detecting.
	virtual TargetType GetTargetType() = 0;
	
	// Gets the combined target info
	virtual Target GetTargetData() = 0;
};

#endif
