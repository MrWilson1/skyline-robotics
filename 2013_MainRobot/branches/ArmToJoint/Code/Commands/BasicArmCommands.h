#ifndef BASIC_ARM_COMMANDS_H
#define BASIC_ARM_COMMANDS_H

#include "BaseCommand.h"
#include "WPILib.h"
#include "../Subsystems/Climber/\ClimberJoints.h"
#include "math.h"
#include "../Misc/Tools.h"

struct ArmData {
	ArmData(BaseSmartJoint *elbow, BaseSmartJoint *shoulder, double elbowLength, double shoulderLength);
	BaseSmartJoint *m_elbow;
	BaseSmartJoint *m_shoulder;
	double m_elbowLength;
	double m_shoulderLength;
};

class SetJointSpeedCommand : public Command {
public:
	SetJointSpeedCommand(BaseJoint *joint, double speed);
	~SetJointSpeedCommand();
	
	void Initialize();
	void Execute();
	bool IsFinished();
	void Interrupted();
	void End();
private:
	BaseJoint *m_joint;
	double m_speed;
};

class SetPolarCommand : public Command {
public:
	SetPolarCommand(ArmData arm, double angle, double magnitude);
	~SetPolarCommand();
	
	void Initialize();
	void Execute();
	bool IsFinished();
	void Interrupted();
	void End();
private:
	ArmData m_armData;
	double m_angle;
	double m_magnitude;
	double m_angleRange;
	double m_magnitudeRange;
};

class SetCartesianCommand : public CommandGroup {
public:
	SetCartesianCommand(ArmData arm, double x, double y);
	~SetCartesianCommand();
};

class SetDegreesCommand : public CommandGroup {
public:
	SetDegreesCommand(ArmData arm, double degrees);
	~SetDegreesCommand();
};

class SetMagnitudeCommand : public CommandGroup {
public:
	SetMagnitudeCommand(ArmData arm, double magnitude);
	~SetMagnitudeCommand();
};

class SetXCommand : public CommandGroup {
public:
	SetXCommand(ArmData arm, double x);
	~SetXCommand();
};

class SetYCommand : public CommandGroup {
public:
	SetYCommand(ArmData arm, double y);
	~SetYCommand();
};

namespace Arm {
	double GetX(ArmData arm);
	double GetY(ArmData arm);
	double GetAngle(ArmData arm);
	double GetMagnitude(ArmData arm);
}


#endif
