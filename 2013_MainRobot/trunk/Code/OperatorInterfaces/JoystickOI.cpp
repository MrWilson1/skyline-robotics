#include "JoystickOI.h"

JoystickOI::JoystickOI(Joystick *rightJoystick, Joystick *leftJoystick, Joystick *twistJoystick, BaseDrive *drive) :
    	BaseOI("Joysticks") {
	m_rightJoystick = rightJoystick;
	m_leftJoystick = leftJoystick;
	m_twistJoystick = twistJoystick;
	m_drive = drive;
	
	m_teleopCommand = new TankDriveCommand(m_drive, this);
}

JoystickOI::~JoystickOI() {
	// empty
}

void JoystickOI::SetupTeleop() {
	SmartDashboard::PutString("JoystickOI status", "Setting up teleop");
	m_drive->SetDefaultCommand(m_teleopCommand);
}

TankValues JoystickOI::GetTankValues() {
	TankValues t;
	t.Left = m_leftJoystick->GetY();
	t.Right = m_rightJoystick->GetY();
	return t;
}

ArcadeValues JoystickOI::GetArcadeValues() {
	ArcadeValues a;
	a.Move = m_twistJoystick->GetY();
	a.Rotate = m_twistJoystick->GetX();
	return a;
}

float JoystickOI::GetAxis(int axis) {
	int new_axis = axis % 2;
	int number = axis / 2;
	return GetAxis(new_axis, number);
}

float JoystickOI::GetAxis(int axis, int joystickNumber) {
	switch (joystickNumber) {
	case 0:
		return m_leftJoystick->GetRawAxis(axis);
	case 1:
		return m_rightJoystick->GetRawAxis(axis);
	case 2:
		return m_twistJoystick->GetRawAxis(axis);
	default:
		SmartDashboard::PutString("GetAxis(), joystickNumber", "Invalid joystick number");
		return 0;
	}
}
