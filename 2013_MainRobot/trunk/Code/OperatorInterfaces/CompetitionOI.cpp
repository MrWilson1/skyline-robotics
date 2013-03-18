#include "CompetitionOI.h"

CompetitionOI::CompetitionOI(XboxController *xbox) :
		BaseOI("CompetitionOI") {
	TankLeftAxis = new TruncatedCurvedAxis(xbox, xbox->LeftY);
	TankRightAxis = new TruncatedCurvedAxis(xbox, xbox->RightY);
	ArcadeMagnitudeAxis = new Axis(xbox, xbox->LeftY);
	ArcadeRotationAxis = new Axis(xbox, xbox->RightX);
	DriveStraightAxis = new Axis(xbox, xbox->LeftY);
	ControlWinchAxis = NULL;
	
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
	delete ControlWinchAxis;
	
	delete DriveStraightButton;
	delete ToggleTransmissionButton;
	delete FireFrisbeeButton;
	delete ClimbRobotButton;
}


XboxTwoJoysticksOI::XboxTwoJoysticksOI(XboxController *xbox, Joystick *leftStick, Joystick *rightStick) :
		BaseOI("XboxTwoJoysticksOI") {
	TankLeftAxis = new Axis(xbox, xbox->LeftY);
	TankRightAxis = new Axis(xbox, xbox->RightY);
	DriveStraightAxis = new Axis(xbox, xbox->LeftY);
	RotateTurretAxis = new Axis(leftStick, Joystick::kDefaultXAxis);
	LiftTurretAxis = new Axis(leftStick, Joystick::kDefaultYAxis);
	ControlWinchAxis = new Axis(rightStick, Joystick::kDefaultYAxis);
	
	DriveStraightButton = new JoystickButton(xbox, xbox->LeftBumper);
	ToggleTransmissionButton = new JoystickButton(xbox, xbox->RightBumper);
	FireFrisbeeButton = new JoystickButton(xbox, xbox->A);
}

XboxTwoJoysticksOI::~XboxTwoJoysticksOI() {
	delete TankLeftAxis;
	delete TankRightAxis;
	delete DriveStraightAxis;
	delete RotateTurretAxis;
	delete LiftTurretAxis;
	delete ControlWinchAxis;
	
	delete DriveStraightButton;
	delete ToggleTransmissionButton;
	delete FireFrisbeeButton;
}



XboxSingleJoystickOI::XboxSingleJoystickOI(XboxController *xbox, Joystick *stick) :
		BaseOI("XboxSingleJoystickOI") {
	TankLeftAxis = new TruncatedCurvedAxis(xbox, xbox->LeftY);
	TankRightAxis = new TruncatedCurvedAxis(xbox, xbox->RightY);
	DriveStraightAxis = new Axis(xbox, xbox->LeftY);
	RotateTurretAxis = new Axis(stick, Joystick::kDefaultXAxis);
	LiftTurretAxis = new CustomRangeAxis(stick, Joystick::kDefaultYAxis, -1.0, 1.0);
	ShooterSpeedAxis = new CustomRangeAxis(stick, Joystick::kDefaultZAxis, 0.5, 1.0);
	
	DriveStraightButton = new JoystickButton(xbox, xbox->LeftBumper);
	ToggleTransmissionButton = new JoystickButton(xbox, xbox->RightBumper);
	FireFrisbeeButton = new JoystickButton(stick, 3);
	LoadFrisbeeButton = new JoystickButton(stick, Joystick::kDefaultTriggerButton);
	
	Winch1Button = new JoystickButton(stick, 7);
	Winch2Button = new JoystickButton(stick, 10);
	ControlWinchButton = new CustomButton::PressTwo(Winch1Button, Winch2Button);
}

XboxSingleJoystickOI::~XboxSingleJoystickOI() {
	delete TankLeftAxis;
	delete TankRightAxis;
	delete DriveStraightAxis;
	delete RotateTurretAxis;
	delete LiftTurretAxis;
	delete ShooterSpeedAxis;
	
	delete DriveStraightButton;
	delete ToggleTransmissionButton;
	delete FireFrisbeeButton;
	delete LoadFrisbeeButton;
	
	delete Winch1Button;
	delete Winch2Button;
	delete ControlWinchButton;
}


ThreeJoystickOI::ThreeJoystickOI(Joystick *rightJoystick, Joystick *leftJoystick, Joystick *twistJoystick) :
		BaseOI("ThreeJoystickOI") {
	TankLeftAxis = new Axis(leftJoystick, Joystick::kDefaultYAxis);
	TankRightAxis = new Axis(rightJoystick, Joystick::kDefaultYAxis);
	ArcadeMagnitudeAxis = new Axis(twistJoystick, Joystick::kDefaultYAxis);
	ArcadeRotationAxis = new Axis(twistJoystick, Joystick::kDefaultTwistAxis);
	DriveStraightAxis = new Axis(leftJoystick, Joystick::kDefaultYAxis);
	ControlWinchAxis = new Axis(rightJoystick, Joystick::kDefaultXAxis);
	
	DriveStraightButton = new JoystickButton(leftJoystick, 3);
	ToggleTransmissionButton = NULL; // don't know yet
	FireFrisbeeButton = new JoystickButton(twistJoystick, 10);
	ClimbRobotButton = new JoystickButton(twistJoystick, 9);
}

ThreeJoystickOI::~ThreeJoystickOI() {
	delete TankLeftAxis;
	delete TankRightAxis;
	delete ArcadeMagnitudeAxis;
	delete ArcadeRotationAxis;
	delete DriveStraightAxis;
	delete ControlWinchAxis;
	
	delete DriveStraightButton;
	delete ToggleTransmissionButton;
	delete FireFrisbeeButton;
	delete ClimbRobotButton;
}
		
