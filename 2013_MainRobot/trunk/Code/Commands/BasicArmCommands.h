#ifndef BASIC_ARM_COMMANDS_H
#define BASIC_ARM_COMMANDS_H

#include "BaseCommand.h"
#include "WPILib.h"
#include "../Subsystems/Climber/ClimberJoints.h"
#include "math.h"
#include "../Misc/Tools.h"
#include "../Subsystems/Controllers/Axis.h"
#include "../Subsystems/Climber/ClimberWinch.h"

class Arm {
public:
	Arm(Joint::BaseSmart *elbow, Joint::BaseSmart *shoulder, double elbowLength, double shoulderLength);
	~Arm();
	Joint::BaseSmart *Elbow;
	Joint::BaseSmart *Shoulder;
	double ElbowLength;
	double ShoulderLength;
	
	double GetX();
	double GetY();
	double GetAngle();
	double GetMagnitude();
};

class SetJointSpeedCommand : public SimpleCommand {
public:
	SetJointSpeedCommand(Joint::Base *joint, double speed);
	~SetJointSpeedCommand();
	void Execute();
private:
	Joint::Base *m_joint;
	double m_speed;
};

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

class SetCartesianCommand : public CommandGroup {
public:
	SetCartesianCommand(Arm arm, double x, double y);
	~SetCartesianCommand();
};

class SetDegreesCommand : public CommandGroup {
public:
	SetDegreesCommand(Arm arm, double degrees);
	~SetDegreesCommand();
};

class SetMagnitudeCommand : public CommandGroup {
public:
	SetMagnitudeCommand(Arm arm, double magnitude);
	~SetMagnitudeCommand();
};

class SetXCommand : public CommandGroup {
public:
	SetXCommand(Arm arm, double x);
	~SetXCommand();
};

class SetYCommand : public CommandGroup {
public:
	SetYCommand(Arm arm, double y);
	~SetYCommand();
};

class ControlWinchManualCommand : public SimpleCommand {
public:
	ControlWinchManualCommand(Winch::Base *winch, Axis *axis);
	~ControlWinchManualCommand();
	
	void Execute();
private:
	Winch::Base *m_winch;
	Axis *m_axis;
};


#endif
