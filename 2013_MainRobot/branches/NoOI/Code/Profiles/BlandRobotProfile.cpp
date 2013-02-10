#include "BlandRobotProfile.h"

BlandRobotProfile::BlandRobotProfile() :
		BaseRobotProfile("BlandRobotProfile") {
	CreateBasicHardwareObjects();
	CreateSubsystems();
	CreateOI();
}

BlandRobotProfile::~BlandRobotProfile() {
	// empty
}

void BlandRobotProfile::CreateBasicHardwareObjects() {
	m_robotDrive = new RobotDrive(
			Ports::DigitalSidecar::Pwm1,
			Ports::DigitalSidecar::Pwm2,
			Ports::DigitalSidecar::Pwm3,
			Ports::DigitalSidecar::Pwm4);
	m_xbox = new XboxController(
			Ports::Computer::Usb1);
}

void BlandRobotProfile::CreateSubsystems() {
	m_drive = new SimpleDrive(m_robotDrive);
}

void BlandRobotProfile::CreateOI() {
	m_OI = new XboxOI(
		m_xbox,
		m_drive);
}

void BlandRobotProfile::RobotInit() {
	// empty
}

void BlandRobotProfile::AutonomousInit() {
	// empty
}

void BlandRobotProfile::TeleopInit() {
	SmartDashboard::PutString("bland status 1", "initializing Telop");
	m_OI->SetupTeleop();
	SmartDashboard::PutString("bland status 2", "done init");
}

