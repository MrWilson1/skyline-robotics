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
	m_leftFrontMotor = new Jaguar(
			Ports::Crio::Module1,
			Ports::DigitalSidecar::Pwm1);
	m_leftBackMotor = new Jaguar(
			Ports::Crio::Module1,
			Ports::DigitalSidecar::Pwm2);
	m_rightFrontMotor = new Jaguar(
			Ports::Crio::Module1,
			Ports::DigitalSidecar::Pwm3);
	m_rightBackMotor = new Jaguar(
			Ports::Crio::Module1,
			Ports::DigitalSidecar::Pwm4);
	
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
	
	m_leftEncoder->SetDistancePerPulse(1.0f / 4500.0f);
	m_rightEncoder->SetDistancePerPulse(1.0f / 4500.0f);
	//m_rightEncoder->SetReverseDirection(true);

	m_leftEncoder->Start();
	m_rightEncoder->Start();
	
	m_xbox = new XboxController(
			Ports::Computer::Usb1);
}

void TerminatorRobotProfile::CreateSubsystems() {
	m_drive = new PidSimpleDrive(
		m_leftFrontMotor,
		m_leftBackMotor,
		m_rightFrontMotor,
		m_rightBackMotor,
		m_leftEncoder,
		m_rightEncoder);
	m_leftTestEncoder = new TestEncoder(m_leftEncoder, "Left Encoder Test");
	m_rightTestEncoder = new TestEncoder(m_rightEncoder, "Right Encoder Test");
}

void TerminatorRobotProfile::CreateOI() {
	m_OI = new TestEncoderOI(
		m_xbox,
		m_drive,
		m_leftTestEncoder,
		m_rightTestEncoder);		
}

void TerminatorRobotProfile::RobotInit() {
	//SmartDashboard::PutData("Scheduler", Scheduler::GetInstance());
	GetWatchdog().SetExpiration(3.0);
}

void TerminatorRobotProfile::AutonomousInit() {
	// empty
}

void TerminatorRobotProfile::TeleopInit() {
	m_OI->SetupTeleop();
}
