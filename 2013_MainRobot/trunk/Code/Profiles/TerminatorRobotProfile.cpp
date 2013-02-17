#include "TerminatorRobotProfile.h"

TerminatorRobotProfile::TerminatorRobotProfile() :
		BaseRobotProfile("TerminatorRobotProfile") {
	CreateBasicHardwareObjects();
	CreateSubsystems();
	CreateOI();
}

TerminatorRobotProfile::~TerminatorRobotProfile() {
	// empty
}

void TerminatorRobotProfile::CreateBasicHardwareObjects() {
	m_leftFrontMotor = new Talon(
			Ports::Crio::Module1,
			Ports::DigitalSidecar::Pwm1);
	m_leftBackMotor = new Talon(
			Ports::Crio::Module1,
			Ports::DigitalSidecar::Pwm2);
	m_rightFrontMotor = new Talon(
			Ports::Crio::Module1,
			Ports::DigitalSidecar::Pwm3);
	m_rightBackMotor = new Talon(
			Ports::Crio::Module1,
			Ports::DigitalSidecar::Pwm4);
	m_leftTread = new Tread(
			m_leftFrontMotor,
			m_leftBackMotor);
	m_rightTread = new Tread(
			m_rightFrontMotor,
			m_rightBackMotor);
	
	m_leftEncoder = new Encoder(
			Ports::Crio::Module1,
			Ports::DigitalSidecar::Gpio6,
			Ports::Crio::Module1,
			Ports::DigitalSidecar::Gpio7);
	m_rightEncoder = new Encoder(
			Ports::Crio::Module1,
			Ports::DigitalSidecar::Gpio8,
			Ports::Crio::Module1,
			Ports::DigitalSidecar::Gpio9);
	
	m_xbox = new XboxController(
			Ports::Computer::Usb1);
}

void TerminatorRobotProfile::CreateSubsystems() {
	// These currently have no effect on the robot whatsoever.
	
	m_drive = new PidSimpleDrive(
		m_leftTread,
		m_rightTread,
		m_leftEncoder,
		m_rightEncoder,
		1.0f / 4134.0f,
		1.0f / 4054.0f,
		240.f / 4134.0f,
		240.f / 4054.0f);
	m_leftTestEncoder = new TestEncoder(m_leftEncoder, "Left Encoder Test");
	m_rightTestEncoder = new TestEncoder(m_rightEncoder, "Right Encoder Test");
}

void TerminatorRobotProfile::CreateOI() {
	m_oi = new SimpleOI(m_xbox);
}

void TerminatorRobotProfile::RobotInit() {
	GetWatchdog().SetExpiration(3.0);
}

void TerminatorRobotProfile::AutonomousInit() {
	// empty
}

void TerminatorRobotProfile::TeleopInit() {
	m_drive->SetDefaultCommand(new TankDriveCommand(
			m_drive,
			m_oi->TankLeftAxis,
			m_oi->TankRightAxis));
	m_leftTestEncoder->SetDefaultCommand(new TestEncoderCommand(
			m_leftTestEncoder, 
			"left Encoder"));
	m_rightTestEncoder->SetDefaultCommand(new TestEncoderCommand(
			m_rightTestEncoder, 
			"right Encoder"));
	m_oi->DriveStraightButton->WhileHeld(new TravelStraightManualCommand(
			m_drive, 
			m_oi->DriveStraightAxis));
}
