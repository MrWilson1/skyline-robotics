#ifndef BASIC_ARM_COMMANDS_H
#define BASIC_ARM_COMMANDS_H
/**
 * \file BasicArmCommands.h
 * 
 * \addtogroup commands
 * \{
 */

#include "BaseCommand.h"
#include "WPILib.h"
#include "../Subsystems/Climber/ClimberJoints.h"
#include "math.h"
#include "../Misc/Tools.h"
#include "../Subsystems/Controllers/Axis.h"
#include "../Subsystems/Climber/ClimberWinch.h"

namespace BasicArmCommand {
/**
 * Arm class consisting of elbow and shoulder joints. Uses polar and Cartesian coordinate systems, 
 * where the shoulder joint is at the origin.
 */
class Arm {
public:
	Arm(Joint::BaseSmart *elbow, Joint::BaseSmart *shoulder, double elbowLength, double shoulderLength);
	~Arm();
	Joint::BaseSmart *Elbow;
	Joint::BaseSmart *Shoulder;
	double ElbowLength;
	double ShoulderLength;
	
	/**
	 * Gets the x-coordinate of the arm hook.
	 */
	double GetX();
	
	/**
	 * Gets the y-coordinate of the arm hook.
	 */
	double GetY();
	
	/**
	 * Gets the angle measured counterclockwise from the x-axis of the vector from the shoulder joint to the arm hook.
	 */
	double GetAngle();
	
	/**
	 * Gets the magnitude of the vector from the shoulder joint to the arm hook.
	 */
	double GetMagnitude();
};

/**
 * Command to set the speed of one of the joints.
 */
class SetJointSpeed : public SimpleCommand {
public:
	SetJointSpeed(Joint::Base *joint, double speed);
	~SetJointSpeed();
	void Execute();
private:
	Joint::Base *m_joint;
	double m_speed;
};

/**
 * Command to set the position of the arm hook using polar coordinates (angle and magnitude).
 */
class SetPolar : public Command {
public:
	SetPolar(Arm arm, double angle, double magnitude);
	~SetPolar();
	
	void Initialize();
	void Execute();
	bool IsFinished();
	void Interrupted();
	void End();
private:
	Arm m_arm;
	double m_angle;
	double m_magnitude;
	double m_angleRange;
	double m_magnitudeRange;
};

/**
 * Command to set the position of the arm hook using Cartesian coordinates.
 */
class SetCartesian :  public CommandGroup {
public:
	SetCartesian(Arm arm, double x, double y);
	~SetCartesian();
};

/**
 * Command to set the angle (measured counterclockwise from the x-axis) of the
 * vector from the shoulder joint to the arm hook.
 */
class SetDegrees :  public CommandGroup {
public:
	SetDegrees(Arm arm, double degrees);
	~SetDegrees();
};

/**
 * Command to set the magnitude of the vector from the shoulder joint to the arm hook.
 */
class SetMagnitude :  public CommandGroup {
public:
	SetMagnitude(Arm arm, double magnitude);
	~SetMagnitude();
};

/**
 * Command to set the x-coordinate of the arm hook.
 */
class SetX :  public CommandGroup {
public:
	SetX(Arm arm, double x);
	~SetX();
};

/**
 * Command to set the y-coordinate of the arm hook.
 */
class SetY :  public CommandGroup {
public:
	SetY(Arm arm, double y);
	~SetY();
};

/**
 * Command to control the winch using an axis.
 */
class ControlWinchManual :  public SimpleCommand {
public:
	ControlWinchManual(Winch::Base *winch, Axis *axis);
	~ControlWinchManual();
	
	void Execute();
private:
	Winch::Base *m_winch;
	Axis *m_axis;
};
}

/**
 * \}
 */
#endif
