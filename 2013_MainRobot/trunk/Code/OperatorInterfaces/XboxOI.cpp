#include "XboxOI.h"

XboxOI::XboxOI(XboxController *xbox, BaseDrive *drive) :
	BaseOI("Xbox Controller")
{
	m_xbox = xbox;
	m_drive = drive;
}

XboxOI::~XboxOI()
{
	// empty
}

void XboxOI::SetupTeleop()
{
	SmartDashboard::PutString("XboxOI status", "Setting up teleop");
	m_spasmButton = new JoystickButton(m_xbox, (int) m_xbox->A);
	m_spasmButton->WhileHeld(new Spasm(m_drive));
	
	m_teleopCommand = new TankDriveCommand(m_drive, this);
	m_teleopCommand->Start();
}

TankValues XboxOI::GetTankValues()
{
	float left = m_xbox->GetAxis(m_xbox->LeftY);
	float right = m_xbox->GetAxis(m_xbox->RightY);
	TankValues t;
	t.Left = left;
	t.Right = right;
	return t;
}

ArcadeValues XboxOI::GetArcadeValues()
{
	float move = m_xbox->GetAxis(m_xbox->LeftY);
	float rotate = m_xbox->GetAxis(m_xbox->RightX);
	ArcadeValues a;
	a.Move = move;
	a.Rotate = rotate;
	return a;
}

