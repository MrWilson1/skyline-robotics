#ifndef FRISBEE_TURRET_H
#define FRISBEE_TURRET_H

#include "WPILib.h"

#include "../BaseSubsystem.h"
#include "FrisbeeAimer.h"


class BaseAxisFrisbeeTurret : public BaseSubsystem {
public: 
	BaseAxisFrisbeeTurret(const char *name);
	virtual ~BaseAxisFrisbeeTurret();
	
	virtual void SetMotor(float speed) = 0;
	virtual void Stop() = 0;
	virtual bool ShouldTurretStop() = 0;
};

class SimpleAxisFrisbeeTurret : public BaseAxisFrisbeeTurret {
public:
	SimpleAxisFrisbeeTurret(SpeedController *motor);
	~SimpleAxisFrisbeeTurret();
	
	void SetMotor(float speed);
	void Stop();
	bool ShouldTurretStop();
	
private: 
	SpeedController *m_motor;
};

class GuardedAxisFrisbeeTurret : public BaseAxisFrisbeeTurret {
public:
	GuardedAxisFrisbeeTurret(SpeedController *motor,  
							 DigitalInput *switch1,
							 DigitalInput *switch2);
	~GuardedAxisFrisbeeTurret();
	
	void SetMotor(float speed);
	void Stop();
	bool ShouldTurretStop();
	
private:
	SpeedController *m_motor; 
	DigitalInput *m_switch1;
	DigitalInput *m_switch2;	
};


////////////////////////
/*
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
	virtual void TurnGivenOffset(Tracking::Offset offset) = 0;
	virtual Tracking::Offset GetCurrentOffset() = 0;
};

class SimpleFrisbeeTurret : public BaseFrisbeeTurret {
public:
	SimpleFrisbeeTurret(SpeedController *horizontalMotor, SpeedController *lateralMotor);
	~SimpleFrisbeeTurret();
	
	void TurnHorizontal(float speed);
	void TurnVertical(float speed);
	void TurnGivenOffset(Tracking::Offset offset);
	Tracking::Offset GetCurrentOffset();

private: 
	SpeedController *m_horizontalMotor; 
	SpeedController *m_lateralMotor;
	Tracking::Offset m_offset;
};

/**
 * Identical to SimpleFrisbeeTurret, but with limit switches.
 * 
 * Assumes limit switches are normally open, so they return 'true' when
 * the switches are not depressed. Assumes the switches return 'false'
 * when depressed or disconnected.
 */

class GuardedFrisbeeTurret : public BaseFrisbeeTurret {
public:
	GuardedFrisbeeTurret(SpeedController *horizontalMotor, 
						 SpeedController *lateralMotor, 
						 DigitalInput *leftSwitch,
						 DigitalInput *rightSwitch,
						 DigitalInput *topSwitch,
						 DigitalInput *bottomSwitch);
	~GuardedFrisbeeTurret();
	
	void TurnHorizontal(float speed);
	void TurnVertical(float speed);
	void TurnGivenOffset(Tracking::Offset offset);
	Tracking::Offset GetCurrentOffset();

private: 
	SpeedController *m_horizontalMotor; 
	SpeedController *m_lateralMotor;
	DigitalInput *m_leftSwitch;
	DigitalInput *m_rightSwitch;
	DigitalInput *m_topSwitch;
	DigitalInput *m_bottomSwitch;
	Tracking::Offset m_offset;
};

/**
 * A class which controllers the shooter turret using
 * a PID loop. 
 * 
 * Assumes that the encoders will return 360 pulses
 * given a full rotation. The encoders must be 
 * calibrated before being passed into this class.
 */
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
	void TurnGivenOffset(Tracking::Offset offset);
	Tracking::Offset GetCurrentOffset();
	
private:
	Tracking::Offset m_offset;
	SpeedController *m_horizontalMotor;
	SpeedController *m_verticalMotor;
	Encoder *m_verticalEncoder;
	Encoder *m_horizontalEncoder;
	
	PIDController *m_horizontalPid;
	PIDController *m_verticalPid;
};


#endif
