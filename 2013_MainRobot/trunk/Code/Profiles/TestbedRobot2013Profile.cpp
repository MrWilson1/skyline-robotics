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
	m_leftMotor = new Talon(
			Ports::Crio::Module1,
			Ports::DigitalSidecar::Pwm1);
	m_rightMotor = new Talon(
			Ports::Crio::Module1,
			Ports::DigitalSidecar::Pwm2);
	m_leftTread = new Tread(m_leftMotor);
	m_rightTread = new Tread(m_rightMotor);
	m_xbox = new XboxController(
			Ports::Computer::Usb1);
	m_compressor = new Compressor(
			Ports::Crio::Module1,
			Ports::DigitalSidecar::Gpio1,
			Ports::Crio::Module1,
			Ports::DigitalSidecar::Relay1);
	m_leftEncoder = new Encoder(
			Ports::Crio::Module1,
			Ports::DigitalSidecar::Gpio2,
			Ports::Crio::Module1,
			Ports::DigitalSidecar::Gpio3);
	m_rightEncoder = new Encoder(
			Ports::Crio::Module1,
			Ports::DigitalSidecar::Gpio4,
			Ports::Crio::Module1,
			Ports::DigitalSidecar::Gpio5);
	m_leftEncoder->Start();
	m_rightEncoder->Start();
	m_rightTransmissionSolenoid = new DoubleSolenoid(
			Ports::Crio::Module1,
			Ports::Crio::SolenoidBreakout1,
			Ports::Crio::SolenoidBreakout2
	);
	m_leftTransmissionSolenoid = new DoubleSolenoid(
		Ports::Crio::Module1,
		Ports::Crio::SolenoidBreakout3,
		Ports::Crio::SolenoidBreakout4
	);
}

void TestbedRobot2013Profile::CreateSubsystems() {
	m_drive = new SimpleDrive(m_leftTread, m_rightTread);
	m_leftTestEncoder = new TestEncoder(
			m_leftEncoder,
			"Left Encoder");
	m_rightTestEncoder = new TestEncoder(
			m_rightEncoder, 
			"Right Encoder");
	m_transmission = new DriveTransmission::Simple(
			m_leftTransmissionSolenoid,
			m_rightTransmissionSolenoid);
}

void TestbedRobot2013Profile::CreateOI() {
	m_oi = new CompetitionOI(m_xbox);
}

void TestbedRobot2013Profile::RobotInit() {
	//m_compressor->Start();
}

void TestbedRobot2013Profile::AutonomousInit() {
	// empty
}

void TestbedRobot2013Profile::TeleopInit() {
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
	m_oi->ToggleTransmissionButton->WhenPressed(new ToggleTransmissionCommand(
			m_transmission));
	m_oi->DriveStraightButton->WhileHeld(new TravelStraightManualCommand(
			m_drive, 
			m_oi->DriveStraightAxis));
}

