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
	Button *ControlWinchBackButton;
	Button *SetTurretHomeButton;
	Button *GotoTurretHomeButton;
		
private:
	Button *Winch1Button;
	Button *Winch2Button;
	
	Button *WinchBack1Button;
	Button *WinchBack2Button;
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


/**
 * \}
 */

#endif
