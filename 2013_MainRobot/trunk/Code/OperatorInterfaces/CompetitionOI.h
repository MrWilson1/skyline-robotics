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
	Axis *ControlWinchAxis;
	
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
	Axis *ControlWinchAxis;
	
	JoystickButton *DriveStraightButton;
	JoystickButton *ToggleTransmissionButton;
	JoystickButton *FireFrisbeeButton;
};


class XboxSingleJoystickOI : public BaseOI{
public:
	XboxSingleJoystickOI(XboxController *xbox, Joystick *stick);
	~XboxSingleJoystickOI();
	
	Axis *TankLeftAxis;
	Axis *TankRightAxis;
	Axis *DriveStraightAxis;
	Axis *RotateTurretAxis;
	Axis *LiftTurretAxis;
	Axis *ControlWinchAxis;
	
	JoystickButton *DriveStraightButton;
	JoystickButton *ToggleTransmissionButton;
	JoystickButton *FireFrisbeeButton;
};


class ThreeJoystickOI : public BaseOI {
public:
	ThreeJoystickOI(Joystick *rightJoystick, Joystick *leftJoystick, Joystick *twistJoystick);
	~ThreeJoystickOI();
	
	Axis *TankLeftAxis;
	Axis *TankRightAxis;
	Axis *ArcadeMagnitudeAxis;
	Axis *ArcadeRotationAxis;
	Axis *DriveStraightAxis;
	Axis *ControlWinchAxis;
	
	JoystickButton *DriveStraightButton;
	JoystickButton *ToggleTransmissionButton;
	JoystickButton *FireFrisbeeButton;
	JoystickButton *ClimbRobotButton;
};

#endif
