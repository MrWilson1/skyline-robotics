#include "TestbedRobot2013Profile.h"

TestbedRobot2013Profile::TestbedRobot2013Profile() :
		BaseRobotProfile("TestbedRobot2013Profile") {
	CreateBasicHardwareObjects();
	CreateSubsystems();
	CreateOI();
}

TestbedRobot2013Profile::~TestbedRobot2013Profile() {
	// empty
}

void TestbedRobot2013Profile::CreateBasicHardwareObjects() {
	m_robotDrive = new RobotDrive(
			Ports::DigitalSidecar::Pwm1,
			Ports::DigitalSidecar::Pwm2,
			Ports::DigitalSidecar::Pwm3,
			Ports::DigitalSidecar::Pwm4);
	m_xbox = new XboxController(
			Ports::Computer::Usb1);
	m_compressor = new Compressor(
			Ports::Crio::Module1,
			Ports::DigitalSidecar::Gpio1,
			Ports::Crio::Module1,
			Ports::DigitalSidecar::Relay1);
}

void TestbedRobot2013Profile::CreateSubsystems() {
	m_drive = new SimpleDrive(m_robotDrive);
}

void TestbedRobot2013Profile::CreateOI() {
	// empty
}

void TestbedRobot2013Profile::RobotInit() {
	m_compressor->Start();
}

void TestbedRobot2013Profile::AutonomousInit() {
	// empty
}

void TestbedRobot2013Profile::TeleopInit() {
	m_drive->SetDefaultCommand(new TankDriveCommand(
			m_drive,
			new Axis(m_xbox, m_xbox->LeftY),
			new Axis(m_xbox, m_xbox->RightY)));
}

