#include "SimpleOi.h"

SimpleOI::SimpleOI(XboxController *xbox) :
		BaseOI("SimpleOI") {
	TankLeftAxis = new Axis(xbox, xbox->LeftY);
	TankRightAxis = new Axis(xbox, xbox->RightY);
	ArcadeMagnitudeAxis = new Axis(xbox, xbox->LeftY);
	ArcadeRotationAxis = new Axis(xbox, xbox->RightX);
	DriveStraightAxis = new Axis(xbox, xbox->LeftY);
	DriveStraightButton = new JoystickButton(xbox, xbox->A);
	
	m_xbox = xbox;
}

SimpleOI::~SimpleOI() {
	delete TankLeftAxis;
	delete TankRightAxis;
	delete ArcadeMagnitudeAxis;
	delete ArcadeRotationAxis;
	delete DriveStraightAxis;
	delete DriveStraightButton;
}

