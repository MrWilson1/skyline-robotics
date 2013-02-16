#include "CompetitionOI.h"

CompetitionOI::CompetitionOI(XboxController *xbox) :
		BaseOI("CompetitionOI") {
	TankLeftAxis = new Axis(xbox, xbox->LeftY);
	TankRightAxis = new Axis(xbox, xbox->RightY);
	ArcadeMagnitudeAxis = new Axis(xbox, xbox->LeftY);
	ArcadeRotationAxis = new Axis(xbox, xbox->RightX);
	DriveStraightAxis = new Axis(xbox, xbox->LeftY);
	ControlArmShoulderAxis = NULL; //new Axis(xbox, xbox->RightY);
	ControlArmElbowAxis = NULL; //new Axis(xbox, xbox->);
	
	DriveStraightButton = new JoystickButton(xbox, xbox->A);
	ToggleTransmissionButton = new JoystickButton(xbox, xbox->B);
	FireFrisbeeButton = new JoystickButton(xbox, xbox->X);
	ClimbRobotButton = new JoystickButton(xbox, xbox->Y);
}

CompetitionOI::~CompetitionOI() {
	delete TankLeftAxis;
	delete TankRightAxis;
	delete ArcadeMagnitudeAxis;
	delete ArcadeRotationAxis;
	delete DriveStraightAxis;
	delete ControlArmShoulderAxis;
	delete ControlArmElbowAxis;
	
	delete DriveStraightButton;
	delete ToggleTransmissionButton;
	delete FireFrisbeeButton;
	delete ClimbRobotButton;
}
		
