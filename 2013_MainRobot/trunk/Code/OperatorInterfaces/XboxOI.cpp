#include "XboxOI.h"

XboxOI::XboxOI(XboxController *xbox, BaseDrive *drive) :
		BaseOI("Xbox Controller") {
	m_xbox = xbox;
	m_drive = drive;
	

	m_spasmButton = new JoystickButton(m_xbox, (int) m_xbox->A);
	m_teleopCommand = new TankDriveCommand(m_drive, this);
}

XboxOI::~XboxOI() {
	// empty
}

void XboxOI::SetupTeleop() {
	SmartDashboard::PutString("XboxOI status", "Setting up teleop");
	m_spasmButton->WhileHeld(new Spasm(m_drive));
	m_teleopCommand->Start();
}

TankValues XboxOI::GetTankValues() {
	float left = GetAxis(m_xbox->LeftY);
	float right = GetAxis(m_xbox->RightY);
	TankValues t;
	t.Left = left;
	t.Right = right;
	return t;
}

ArcadeValues XboxOI::GetArcadeValues() {
	float move = GetAxis(m_xbox->LeftY);
	float rotate = GetAxis(m_xbox->RightX);
	ArcadeValues a;
	a.Move = move;
	a.Rotate = rotate;
	return a;
}

float XboxOI::GetAxis(int axis) {
	float value = m_xbox->GetRawAxis(axis);
	// Apply filters to value here if necessary
	return value;
}
