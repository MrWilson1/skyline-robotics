#include "BlandRobotProfile.h"

BlandRobotProfile::BlandRobotProfile()
{
	m_robotDrive = new RobotDrive(
		Ports::DigitalSidecar::Pwm1,
		Ports::DigitalSidecar::Pwm2,
		Ports::DigitalSidecar::Pwm3,
		Ports::DigitalSidecar::Pwm4);
	m_xbox = new XboxController(
			Ports::Usb1);
	
	m_drive = new SimpleDrive(m_robotDrive);
	
	m_OI = new XboxOI(
		m_xbox,
		m_drive);
}

BlandRobotProfile::~BlandRobotProfile()
{
	// empty
}

void BlandRobotProfile::RobotInit()
{
	// empty
}

void BlandRobotProfile::AutonomousInit()
{
	// empty
}

void BlandRobotProfile::AutonomousPeriodic()
{
	Scheduler::GetInstance()->Run();
}

void BlandRobotProfile::TeleopInit()
{
	m_OI->SetupTelop();
}

void BlandRobotProfile::TeleopPeriodic()
{
	Scheduler::GetInstance()->Run();
}

void BlandRobotProfile::TestPeriodic()
{
	lw->Run();
}
