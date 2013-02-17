#ifndef COMPETITION_OI_H
#define COMPETITION_OI_H

#include "WPILib.h"
#include "BaseOI.h"
#include "../Subsystems/Controllers/XboxController.h"
#include "../Subsystems/Controllers/Axis.h"

class CompetitionOI : public BaseOI {
public:
	CompetitionOI(XboxController *xbox);
	~CompetitionOI();
	
	Axis *TankLeftAxis;
	Axis *TankRightAxis;
	Axis *ArcadeMagnitudeAxis;
	Axis *ArcadeRotationAxis;
	Axis *DriveStraightAxis;
	Axis *ControlArmShoulderAxis;
	Axis *ControlArmElbowAxis;
	
	JoystickButton *DriveStraightButton;
	JoystickButton *ToggleTransmissionButton;
	JoystickButton *FireFrisbeeButton;
	JoystickButton *ClimbRobotButton;
};


class XboxTwoJoysticksOI : public BaseOI{
public:
	XboxTwoJoysticksOI(XboxController *xbox, Joystick *leftStick, Joystick *rightStick);
	~XboxTwoJoysticksOI();
	
	Axis *TankLeftAxis;
	Axis *TankRightAxis;
	Axis *DriveStraightAxis;
	Axis *RotateTurretAxis;
	Axis *LiftTurretAxis;
	Axis *ShoulderAxis;
	Axis *ElbowAxis;
	
	JoystickButton *DriveStraightButton;
	JoystickButton *ToggleTransmissionButton;
	JoystickButton *FireFrisbeeButton;
};

class CompetitionJoystickOI : public BaseOI {
public:
	CompetitionJoystickOI(Joystick *rightJoystick, Joystick *leftJoystick, Joystick *twistJoystick);
	~CompetitionJoystickOI();
	
	Axis *TankLeftAxis;
	Axis *TankRightAxis;
	Axis *ArcadeMagnitudeAxis;
	Axis *ArcadeRotationAxis;
	Axis *DriveStraightAxis;
	Axis *ControlArmShoulderAxis;
	Axis *ControlArmElbowAxis;
	
	JoystickButton *DriveStraightButton;
	JoystickButton *ToggleTransmissionButton;
	JoystickButton *FireFrisbeeButton;
	JoystickButton *ClimbRobotButton;
};

#endif
