#ifndef FRISBEE_TURRET_H
#define FRISBEE_TURRET_H

#include "WPILib.h"

#include "FrisbeeAimer.h"

/**
 * Base class for a frisbee turret. Responsible for aiming the 
 * shooter at a certain angle.
 * 
 * Note: this interface may change substantially over time.
 */
class BaseFrisbeeTurret : public Subsystem {
public:
	BaseFrisbeeTurret(const char *name);
	virtual ~BaseFrisbeeTurret();
	
	virtual void TurnHorizontal(float degrees) = 0;
	virtual void TurnVertical(float degrees) = 0;
	virtual void TurnGivenOffset(Offset offset) = 0;
	virtual Offset ReportCurrentOffset() = 0;
};



#endif
