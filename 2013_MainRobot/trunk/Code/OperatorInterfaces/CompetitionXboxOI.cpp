#include "CompetitionXboxOI.h"

CompetitionXboxOI::CompetitionXboxOI(
		XboxController *xbox, 
		BaseDrive *drive,
		BaseFrisbeeLoader *loader,
		BaseFrisbeeAimer *aimer,
		BaseFrisbeeTurret *turret,
		BaseFrisbeeShooter *shooter) :
		BaseOI("CompetitionXboxOI") {
	m_xbox = xbox;
	m_drive = drive;
	m_loader = loader;
	m_aimer = aimer;
	m_turret = turret;
	m_shooter = shooter;

	m_fireButton = new JoystickButton(m_xbox, m_xbox->A);
	m_teleopCommand = new TankDriveCommand(m_drive, this);
	m_loadAndFireCommand = new LoadAndFireCommand(m_loader,  m_aimer, m_turret, m_shooter);
	m_autoAimTargetCommand = new AimTurretCommand(m_aimer, m_turret, Tracking::ClosestOffset, 3.0);
	// Will stop if the shooter is within 3 degrees of the centerpoint of the target
}

CompetitionXboxOI::~CompetitionXboxOI() {
	// empty
}

void CompetitionXboxOI::SetupTeleop() {
	m_fireButton->WhenPressed(m_loadAndFireCommand);
	m_drive->SetDefaultCommand(m_teleopCommand);
	m_turret->SetDefaultCommand(m_autoAimTargetCommand);
}

TankValues CompetitionXboxOI::GetTankValues() {
	float left = GetAxis(m_xbox->LeftY);
	float right = GetAxis(m_xbox->RightY);
	TankValues t;
	t.Left = left;
	t.Right = right;
	return t;
}

ArcadeValues CompetitionXboxOI::GetArcadeValues() {
	float move = GetAxis(m_xbox->LeftY);
	float rotate = GetAxis(m_xbox->RightX);
	ArcadeValues a;
	a.Move = move;
	a.Rotate = rotate;
	return a;
}

float CompetitionXboxOI::GetAxis(int axis) {
	float value = m_xbox->GetRawAxis(axis);
	// Apply filters to value here if necessary
	return value;
}

float CompetitionXboxOI::GetAxis(int axis, int joystickNumber) {
	float value = m_xbox->GetRawAxis(axis);
	// Apply filters to value here if necessary
	return value;
}

