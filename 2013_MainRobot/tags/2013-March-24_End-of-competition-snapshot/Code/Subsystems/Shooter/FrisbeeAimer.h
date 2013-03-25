#ifndef FRISBEE_AIMER_H
#define FRISBEE_AIMER_H

#include <map>
#include <string>

#include "WPILib.h"
#include "WPILib/networktables2/type/NumberArray.h"

#include "../BaseSubsystem.h"
#include "../../Misc/Tools.h"

/**
 * Namespace for tracking-related things.
 */
namespace Tracking {
	/**
	 * The different types of targets the robot can aim for.
	 */
	enum TargetType {
		None,
		Unknown,
		Test,
		Low,
		MiddleLeft,
		MiddleRight,
		High,
		Pyramid,
		ClosestOffset,
		ClosestDistance
	};

	/**
	 * The offset in degrees needed to turn the shooter to point at the
	 * target. If the shooter needs to move 3 degrees up, 5 degrees to
	 * the left, the value returned would be {-5, 3}
	 */
	struct Offset {
		Offset();
		Offset(float x, float y);
		float XOffset;
		float YOffset;
	};
	
	/**
	 * A struct representing a single target.
	 */
	struct Target {
		Target();
		Target(TargetType type, Offset shooterOffset, double distanceInInches);
		TargetType Type;
		Offset ShooterOffset;
		double DistanceInInches;
	};
	
	float FindTargetMagnitude(Target target);

	typedef std::map<TargetType, Target> TargetMap;
}

/**
 * Namespace for FrisbeeAimer.
 */
namespace FrisbeeAimer {
/**
 * A base class to find the targets to aim for.
 * 
 * Note: this interface may change substantially over time.
 */
class Base : public BaseSubsystem {
public:
	Base(const char *name);
	virtual ~Base();

	/**
	 * Return all targets in sight
	 */
	virtual Tracking::TargetMap GetAllTargets() = 0;
	
	/**
	 * Gets the target info for the highest target
	 */
	virtual Tracking::Target GetHighTarget() = 0;
	
	/**
	 * Gets the target info for the middle target to the left
	 * when facing the goals
	 */
	virtual Tracking::Target GetMiddleLeftTarget() = 0;
	
	/**
	 * Gets the target info for the middle target tp tje rogjt
	 * when facing the goals
	 */
	virtual Tracking::Target GetMiddleRightTarget() = 0;
	
	/**
	 * Gets the target info for the low target
	 */
	virtual Tracking::Target GetLowTarget() = 0;
	
	/**
	 * Gets the target info for the target which is 
	 * physically the closest to the robot. Avoid using this,
	 * since the distance is frequently inaccurate for targets
	 * the robot is not pointing straight at/is tilted at
	 * too extreme of an angle.
	 */
	virtual Tracking::Target GetClosestTargetByDistance() = 0;
	
	/**
	 * Gets the target which requires the least turning
	 * for the FrisbeeTurret
	 */
	virtual Tracking::Target GetClosestTargetByOffset() = 0;
};


/**
 * A class which interfaces via NetworkTables to get vision
 * data from the client computer.
 * 
 * This class polls a NetworkTable named 'VisionTable' which
 * must contain the following keys:
 * 
 * -	high_offset_X
 * -	high_offset_y
 * -	high_distance
 * -	middle_left_offset_x
 * -	middle_left_offset_y
 * -	middle_left_distance
 * -	middle_right_offset_x
 * -	middle_right_offset_y
 * -	middle_left_distance
 * -	low_offset_x
 * -	low_offset_y
 * -	low_distance
 * 
 * Most of the processing is done on the client end. This 
 * class simply recieves the values, repackages them, and makes
 * them available.
 */
class VisionTables : public Base {
public:
	VisionTables();
	VisionTables(std::string tableName);
	~VisionTables();
	
	Tracking::TargetMap GetAllTargets();
	Tracking::Target GetHighTarget();
	Tracking::Target GetMiddleLeftTarget();
	Tracking::Target GetMiddleRightTarget();
	Tracking::Target GetLowTarget();
	Tracking::Target GetClosestTargetByDistance();
	Tracking::Target GetClosestTargetByOffset();
	
private:
	Tracking::Target GetTarget(std::string key);
	NetworkTable *m_visionTable;
	std::string m_tableName;
};

class Test : public BaseSubsystem {
public:
	Test(const char *name);
	~Test();
	
	void Run();
};
}

#endif
