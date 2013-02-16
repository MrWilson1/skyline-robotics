#ifndef SIMPLE_OI_H
#define SIMPLE_OI_H

#include "WPILib.h"
#include "BaseOI.h"

#include "../Subsystems/Controllers/Axis.h"
#include "../Subsystems/Controllers/XboxController.h"

class SimpleOI : public BaseOI {
public:
	SimpleOI(XboxController *xbox);
	~SimpleOI();
	
	Axis *TankLeftAxis;
	Axis *TankRightAxis;
	Axis *ArcadeMagnitudeAxis;
	Axis *ArcadeRotationAxis;
	Axis *DriveStraightAxis;
	JoystickButton *DriveStraightButton;
	
private:
	XboxController *m_xbox;
};

#endif
