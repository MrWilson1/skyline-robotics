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

#endif
