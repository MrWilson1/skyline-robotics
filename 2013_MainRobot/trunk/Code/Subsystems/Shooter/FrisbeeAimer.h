#ifndef FRISBEE_AIMER_H
#define FRISBEE_AIMER_H

#include <map>
#include <string>

#include "WPILib.h"
#include "WPILib/networktables2/type/NumberArray.h"

#include "../BaseSubsystem.h"
#include "../../Misc/Tools.h"

namespace Tracking {
	/**
	 * The 5 different types of targets the robot can aim for.
	 */
	enum TargetType {
		None,
		Unknown,
		Test,
		Low,
		MediumLeft,
		MediumRight,
		High,
		Pyramid
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
 * A base class to find the targets to aim for.
 * 
 * Note: this interface may change substantially over time.
 */
class BaseFrisbeeAimer : public BaseSubsystem {
public:
	BaseFrisbeeAimer(const char *name);
	virtual ~BaseFrisbeeAimer();

	/**
	 * Return all targets in sight
	 */
	virtual Tracking::TargetMap GetAllTargets() = 0;
	
	/**
	 * Gets the target info for the highest target
	 */
	virtual Tracking::Target GetHighTarget() = 0;
	
	/**
	 * Gets the target info for the medium target to the left
	 * when facing the goals
	 */
	virtual Tracking::Target GetMediumLeftTarget() = 0;
	
	/**
	 * Gets the target info for the medium target tp tje rogjt
	 * when facing the goals
	 */
	virtual Tracking::Target GetMediumRightTarget() = 0;
	
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
 * -	medium_left_offset_x
 * -	medium_left_offset_y
 * -	medium_left_distance
 * -	medium_right_offset_x
 * -	medium_right_offset_y
 * -	medium_left_distance
 * -	low_offset_x
 * -	low_offset_y
 * -	low_distance
 * 
 * Most of the processing is done on the client end. This 
 * class simply recieves the values, repackages them, and makes
 * them available.
 */
class VisionTablesFrisbeeAimer : public BaseFrisbeeAimer {
public:
	VisionTablesFrisbeeAimer();
	VisionTablesFrisbeeAimer(std::string tableName);
	~VisionTablesFrisbeeAimer();
	
	Tracking::TargetMap GetAllTargets();
	Tracking::Target GetHighTarget();
	Tracking::Target GetMediumLeftTarget();
	Tracking::Target GetMediumRightTarget();
	Tracking::Target GetLowTarget();
	Tracking::Target GetClosestTargetByDistance();
	Tracking::Target GetClosestTargetByOffset();
	
private:
	Tracking::Target GetTarget(std::string key);
	NetworkTable *m_visionTable;
	std::string m_tableName;
};

class TestAimer : public BaseSubsystem {
public:
	TestAimer(const char *name);
	~TestAimer();
	
	void Run();
};

#endif
