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

/**
 * Arm class consisting of elbow and shoulder joints. Uses polar and Cartesian coordinate systems, 
 * where the shoulder joint is at the origin.
 */
class Arm {
public:
	Arm(BaseSmartJoint *elbow, BaseSmartJoint *shoulder, double elbowLength, double shoulderLength);
	~Arm();
	BaseSmartJoint *Elbow;
	BaseSmartJoint *Shoulder;
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
class SetJointSpeedCommand : public SimpleCommand {
public:
	SetJointSpeedCommand(BaseJoint *joint, double speed);
	~SetJointSpeedCommand();
	void Execute();
private:
	BaseJoint *m_joint;
	double m_speed;
};

/**
 * Command to set the position of the arm hook using polar coordinates (angle and magnitude).
 */
class SetPolarCommand : public Command {
public:
	SetPolarCommand(Arm arm, double angle, double magnitude);
	~SetPolarCommand();
	
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
class SetCartesianCommand : public CommandGroup {
public:
	SetCartesianCommand(Arm arm, double x, double y);
	~SetCartesianCommand();
};

/**
 * Command to set the angle (measured counterclockwise from the x-axis) of the
 * vector from the shoulder joint to the arm hook.
 */
class SetDegreesCommand : public CommandGroup {
public:
	SetDegreesCommand(Arm arm, double degrees);
	~SetDegreesCommand();
};

/**
 * Command to set the magnitude of the vector from the shoulder joint to the arm hook.
 */
class SetMagnitudeCommand : public CommandGroup {
public:
	SetMagnitudeCommand(Arm arm, double magnitude);
	~SetMagnitudeCommand();
};

/**
 * Command to set the x-coordinate of the arm hook.
 */
class SetXCommand : public CommandGroup {
public:
	SetXCommand(Arm arm, double x);
	~SetXCommand();
};

/**
 * Command to set the y-coordinate of the arm hook.
 */
class SetYCommand : public CommandGroup {
public:
	SetYCommand(Arm arm, double y);
	~SetYCommand();
};

/**
 * Command to control the winch using an axis.
 */
class ControlWinchManualCommand : public SimpleCommand {
public:
	ControlWinchManualCommand(BaseWinch *winch, Axis *axis);
	~ControlWinchManualCommand();
	
	void Execute();
private:
	BaseWinch *m_winch;
	Axis *m_axis;
};

/**
 * \}
 */
#endif
