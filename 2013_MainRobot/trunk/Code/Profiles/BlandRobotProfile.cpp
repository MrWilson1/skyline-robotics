#include "BlandRobotProfile.h"

BlandRobotProfile::BlandRobotProfile() :
		IterativeRobot()
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
	
	lw = LiveWindow::GetInstance();
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
	SmartDashboard::PutString("bland status 1", "initializing Telop");
	m_OI->SetupTeleop();
	SmartDashboard::PutString("bland status 2", "done init");
}

void BlandRobotProfile::TeleopPeriodic()
{
	SmartDashboard::PutString("bland status 3", "running teleop");
	Scheduler::GetInstance()->Run();
}

void BlandRobotProfile::TestPeriodic()
{
	lw->Run();
}
