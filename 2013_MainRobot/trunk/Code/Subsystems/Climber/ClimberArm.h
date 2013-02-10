#ifndef CLIMBER_ARM_H
#define CLIMBER_ARM_H

#include <cmath>
#include "WPILib.h"
#include "../BaseSubsystem.h"
#include "ClimberJoints.h"
#include "../../Misc/Tools.h"

/**
 * \brief Base class for all climber arms.
 */
class BaseClimberArm : public BaseSubsystem {
public:
	BaseClimberArm(const char *name);
	virtual ~BaseClimberArm();

	virtual void SetPolar(double degrees, double magnitude) = 0;
	virtual void SetCartesian(double x, double y) = 0;
	virtual void SetDegrees(double degrees) = 0;
	virtual void SetMagnitude(double magnitude) = 0;
	virtual void SetX(double x) = 0;
	virtual void SetY(double y) = 0;
	virtual double GetAngle() = 0;
	virtual double GetMagnitude() = 0;
	virtual double GetX() = 0;
	virtual double GetY() = 0;
};

/**
 * \brief Arm that uses a bungee cord and a winch to climb.
 */
class BungeeWinchClimberArm : public BaseClimberArm {
public:
	BungeeWinchClimberArm(BaseClimberJoint *elbow, BaseClimberJoint *shoulder, double elbowLength, double shoulderLength);
	~BungeeWinchClimberArm();
	
	void SetPolar(double degrees, double magnitude);
	void SetCartesian(double x, double y);
	void SetDegrees(double degrees);
	void SetMagnitude(double magnitude);
	void SetX(double x);
	void SetY(double y);
	double GetAngle();
	double GetMagnitude();
	double GetX();
	double GetY();
private:
	BaseClimberJoint *m_elbow;
	BaseClimberJoint *m_shoulder;
	double m_elbowLength;
	double m_shoulderLength;
};

#endif
