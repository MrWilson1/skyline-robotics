#include "CompetitionOI.h"

CompetitionOI::CompetitionOI(XboxController *xboxDrive, XboxController *xboxShooter, Joystick *joystick) :
		BaseOI("CompetitionOI") {
		TankLeftAxis = new TruncatedCurvedAxis(xboxDrive, xboxDrive->LeftY);
		TankRightAxis = new TruncatedCurvedAxis(xboxDrive, xboxDrive->RightY);
		ArcadeMagAxis = new TruncatedCurvedAxis(xboxDrive, xboxDrive->LeftY);
		ArcadeRotateAxis = new TruncatedCurvedAxis(xboxDrive, xboxDrive->RightX, true);
		DriveStraightAxis = new Axis(xboxDrive, xboxDrive->LeftY);

		RotateTurretAxis = new Axis(xboxShooter, xboxShooter->RightX);
		LiftTurretAxis = new CustomRangeAxis(xboxShooter, xboxShooter->LeftY, -1.0, 1.0);
		DriveStraightButton = new JoystickButton(xboxDrive, xboxDrive->LeftBumper);
		ToggleTransmissionButton = new JoystickButton(xboxDrive, xboxDrive->RightBumper);
		FireFrisbeeButton = new JoystickButton(xboxShooter, xboxShooter->LeftBumper);
		LoadFrisbeeButton = new JoystickButton(xboxShooter, xboxShooter->RightBumper); //CustomButton::XboxTrigger(xboxShooter, xboxShooter->Bumper);
		SetTurretHomeButton = new JoystickButton(xboxShooter, xboxShooter->X);
		GotoTurretHomeButton = new JoystickButton(xboxShooter, xboxShooter->Y);
		
		ShooterSpeedAxis = new CustomRangeAxis(joystick, Joystick::kDefaultZAxis, 0.5, 1.0);
		
		ControlWinchButton = new JoystickButton(joystick, 7);
		ControlWinchBackButton = new JoystickButton(joystick, 10);
		
		/*
		Winch1Button = new JoystickButton(joystick, 7);
		Winch2Button = new JoystickButton(joystick, 10);
		ControlWinchButton = new CustomButton::PressTwo(Winch1Button, Winch2Button);
		
		
		WinchBack1Button = new JoystickButton(joystick, 6);
		WinchBack2Button = new JoystickButton(joystick, 11);
		ControlWinchBackButton = new CustomButton::PressTwo(WinchBack1Button, WinchBack2Button);
		*/
}

CompetitionOI::~CompetitionOI() {
	delete TankLeftAxis;
	delete TankRightAxis;
	delete ArcadeMagAxis;
	delete ArcadeRotateAxis;
	delete DriveStraightAxis;
	delete RotateTurretAxis;
	delete LiftTurretAxis;
	delete ShooterSpeedAxis;
	
	delete DriveStraightButton;
	delete ToggleTransmissionButton;
	delete FireFrisbeeButton;
	delete LoadFrisbeeButton;
	delete SetTurretHomeButton;
	delete GotoTurretHomeButton;
	
	//delete Winch1Button;
	//delete Winch2Button;
	delete ControlWinchButton;
	

	//delete WinchBack1Button;
	//delete WinchBack2Button;
	delete ControlWinchBackButton;
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


