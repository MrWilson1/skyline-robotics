#ifndef CLIMBER_ARM_H
#define CLIMBER_ARM_H

#include <cmath>
#include "WPILib.h"
#include "../BaseSubsystem.h"
#include "ClimberJoints.h"
#include "../../Misc/Tools.h"

/**
 * \brief Base class for all climber arms. Uses polar and Cartesian coordinates to move and locate arm, with the shoulder as the origin.
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
	/**
	 * \param[in] elbow Pointer to the elbow
	 * \param[in] shoulder Pointer to the shoulder
	 * \param[in] shoulderLength Distance between shoulder and elbow
	 */
	BungeeWinchClimberArm(BaseClimberJoint *elbow, BaseClimberJoint *shoulder, double elbowLength, double shoulderLength);
	~BungeeWinchClimberArm();
	
	/**
	 * \brief Moves arm hook to polar coordinates.
	 */
	void SetPolar(double degrees, double magnitude);
	
	/**
	 * \brief Moves arm hook to Cartesian coordinates.
	 */
	void SetCartesian(double x, double y);
	
	/**
	 * \brief Sets angle between x-axis and line running from origin (shoulder) to hook. Keeps the magnitude the same.
	 */
	void SetDegrees(double degrees);
	
	/**
	 * \brief Sets line running from shoulder to hook to specific length. Keeps the angle the same.
	 */
	void SetMagnitude(double magnitude);
	
	/**
	 * \brief Sets x-coordinate of arm hook.
	 */
	void SetX(double x);
	
	/**
	 * \brief Sets y-coordinate of arm hook.
	 */
	void SetY(double y);
	
	/**
	 * \brief Gets angle between x-axis and line from shoulder to hook.
	 */
	double GetAngle();
	
	/**
	 * \brief Gets magnitude of line from shoulder to hook.
	 */
	double GetMagnitude();
	
	/**
	 * \brief Gets x-coordinate of hook.
	 */
	double GetX();
	
	/**
	 * \brief Sets y-coordinate of hook.
	 */
	double GetY();
private:
	BaseClimberJoint *m_elbow;
	BaseClimberJoint *m_shoulder;
	double m_elbowLength;
	double m_shoulderLength;
};

#endif
