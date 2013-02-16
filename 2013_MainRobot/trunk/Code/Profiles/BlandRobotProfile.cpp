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
	m_oi = new SimpleOI(m_xbox);
}

void BlandRobotProfile::RobotInit() {
	// empty
}

void BlandRobotProfile::AutonomousInit() {
	// empty
}

void BlandRobotProfile::TeleopInit() {
	m_drive->SetDefaultCommand(new TankDriveCommand(
			m_drive, 
			m_oi->TankLeftAxis,
			m_oi->TankRightAxis));
}

