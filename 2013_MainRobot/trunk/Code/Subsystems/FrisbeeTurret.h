#ifndef FRISBEE_TURRET_H
#define FRISBEE_TURRET_H

#include "WPILib.h"

#include "BaseSubsystem.h"
#include "FrisbeeAimer.h"

/**
 * Base class for a frisbee turret. Responsible for aiming the 
 * shooter at a certain angle.
 * 
 * Note: this interface may change substantially over time.
 */
class BaseFrisbeeTurret : public BaseSubsystem {
public:
	BaseFrisbeeTurret(const char *name);
	virtual ~BaseFrisbeeTurret();
	
	virtual void TurnHorizontal(float degrees) = 0;
	virtual void TurnVertical(float degrees) = 0;
	virtual void TurnGivenOffset(Offset offset) = 0;
	virtual Offset ReportCurrentOffset() = 0;
};


class PidFrisbeeTurret : public BaseFrisbeeTurret {
public:
	PidFrisbeeTurret(
			SpeedController *horizontalMotor, 
			SpeedController *verticalMotor,
			Encoder *horizontalEncoder,
			Encoder *verticalEncoder);
	~PidFrisbeeTurret();
	
	void TurnHorizontal(float degrees);
	void TurnVertical(float degrees);
	void TurnGivenOffset(Offset offset);
	Offset ReportCurrentOffset();
	
private:
	Offset m_offset;
	SpeedController *m_horizontalMotor;
	SpeedController *m_verticalMotor;
	Encoder *m_verticalEncoder;
	Encoder *m_horizontalEncoder;
};

#endif
