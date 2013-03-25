#ifndef SIMPLE_OI_H
#define SIMPLE_OI_H
/*
 * \file SimpleOI.h
 * 
 * \addtogroup oi
 * \{
 */

#include "WPILib.h"
#include "BaseOI.h"

#include "../Subsystems/Controllers/Axis.h"
#include "../Subsystems/Controllers/XboxController.h"

class SimpleOI : public BaseOI {
public:
	SimpleOI(XboxController *xbox);
	~SimpleOI();
	
	BaseAxis *TankLeftAxis;
	BaseAxis *TankRightAxis;
	BaseAxis *ArcadeMagnitudeAxis;
	BaseAxis *ArcadeRotationAxis;
	BaseAxis *DriveStraightAxis;
	JoystickButton *DriveStraightButton;
	
private:
	XboxController *m_xbox;
};

/*
 * \}
 */

#endif
