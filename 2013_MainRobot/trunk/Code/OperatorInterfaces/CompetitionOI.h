#ifndef COMPETITION_OI_H
#define COMPETITION_OI_H
/**
 * \file CompetitionOI.h
 * 
 * \addtogroup oi
 * \{
 */

#include "WPILib.h"
#include "BaseOI.h"
#include "../Subsystems/Controllers/XboxController.h"
#include "../Subsystems/Controllers/Axis.h"
#include "../Subsystems/Controllers/CustomButtons.h"

/**
 * OI for the competition.
 */
class CompetitionOI : public BaseOI {
public:
	CompetitionOI(XboxController *xboxDrive, XboxController *xboxShooter, Joystick *joystick);
	~CompetitionOI();
	
	BaseAxis *TankLeftAxis;
		BaseAxis *TankRightAxis;
		BaseAxis *ArcadeMagAxis;
		BaseAxis *ArcadeRotateAxis;
		BaseAxis *DriveStraightAxis;
		BaseAxis *RotateTurretAxis;
		BaseAxis *LiftTurretAxis;
		BaseAxis *ShooterSpeedAxis;
		
		Button *DriveStraightButton;
		Button *ToggleTransmissionButton;
		Button *FireFrisbeeButton;
		Button *LoadFrisbeeButton;
		Button *ControlWinchButton;
		
private:
	Button *Winch1Button;
	Button *Winch2Button;
};

/**
 * OI using one Xbox controller and two joysticks.
 */
class XboxTwoJoysticksOI : public BaseOI{
public:
	XboxTwoJoysticksOI(XboxController *xbox, Joystick *leftStick, Joystick *rightStick);
	~XboxTwoJoysticksOI();
	
	BaseAxis *TankLeftAxis;
	BaseAxis *TankRightAxis;
	BaseAxis *DriveStraightAxis;
	BaseAxis *RotateTurretAxis;
	BaseAxis *LiftTurretAxis;
	BaseAxis *ControlWinchAxis;
	
	Button *DriveStraightButton;
	Button *ToggleTransmissionButton;
	Button *FireFrisbeeButton;
};



class XboxSingleJoystickOI : public BaseOI{
public:
	XboxSingleJoystickOI(XboxController *xbox, Joystick *stick);
	~XboxSingleJoystickOI();
	
	BaseAxis *TankLeftAxis;
	BaseAxis *TankRightAxis;
	BaseAxis *DriveStraightAxis;
	BaseAxis *RotateTurretAxis;
	BaseAxis *LiftTurretAxis;
	BaseAxis *ShooterSpeedAxis;
	
	Button *DriveStraightButton;
	Button *ToggleTransmissionButton;
	Button *FireFrisbeeButton;
	Button *LoadFrisbeeButton;
	Button *ControlWinchButton;
	
private:
	Button *Winch1Button;
	Button *Winch2Button;
};


class ThreeJoystickOI : public BaseOI {
public:
	ThreeJoystickOI(Joystick *rightJoystick, Joystick *leftJoystick, Joystick *twistJoystick);
	~ThreeJoystickOI();
	
	BaseAxis *TankLeftAxis;
	BaseAxis *TankRightAxis;
	BaseAxis *ArcadeMagnitudeAxis;
	BaseAxis *ArcadeRotationAxis;
	BaseAxis *DriveStraightAxis;
	BaseAxis *ControlWinchAxis;
	
	Button *DriveStraightButton;
	Button *ToggleTransmissionButton;
	Button *FireFrisbeeButton;
	Button *ClimbRobotButton;
};

/**
 * \}
 */

#endif
