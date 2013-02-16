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

CompetitionJoystickOI::CompetitionJoystickOI(Joystick *rightJoystick, Joystick *leftJoystick, Joystick *twistJoystick) :
		BaseOI("CompetitionJoystickOI") {
	TankLeftAxis = new Axis(leftJoystick, Joystick::kDefaultYAxis);
	TankRightAxis = new Axis(rightJoystick, Joystick::kDefaultYAxis);
	ArcadeMagnitudeAxis = new Axis(twistJoystick, Joystick::kDefaultYAxis);
	ArcadeRotationAxis = new Axis(twistJoystick, Joystick::kDefaultTwistAxis);
	DriveStraightAxis = new Axis(leftJoystick, Joystick::kDefaultYAxis);
	ControlArmShoulderAxis = NULL; // don't know yet
	ControlArmElbowAxis = NULL; // don't know yet
	
	DriveStraightButton = new JoystickButton(leftJoystick, 3);
	ToggleTransmissionButton = NULL; // don't know yet
	FireFrisbeeButton = new JoystickButton(twistJoystick, 10);
	ClimbRobotButton = new JoystickButton(twistJoystick, 9);
}

CompetitionJoystickOI::~CompetitionJoystickOI() {
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
		
