#include "MainRobot2013DefenseProfile.h"

MainRobot2013DefenseProfile::MainRobot2013DefenseProfile() :
		BaseRobotProfile("MainRobot2013DefenseProfile") {
	CreateBasicHardwareObjects();
	CreateSubsystems();
	CreateOI();
}

MainRobot2013DefenseProfile::~MainRobot2013DefenseProfile() {
	// empty
}

void MainRobot2013DefenseProfile::CreateBasicHardwareObjects() {
	// Drive
	m_leftMotor = new Talon(
			Ports::Crio::Module1,
			Ports::DigitalSidecar::Pwm1);
	m_rightMotor = new Talon(
			Ports::Crio::Module1,
			Ports::DigitalSidecar::Pwm2);
	m_leftTread = new Tread(m_leftMotor);
	m_rightTread = new Tread(m_rightMotor);
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
	
	// Transmission
	m_rightTransmissionSolenoid = new DoubleSolenoid(
			Ports::Crio::Module1,
			Ports::Crio::SolenoidBreakout1,
			Ports::Crio::SolenoidBreakout2);
	m_leftTransmissionSolenoid = new DoubleSolenoid(
			Ports::Crio::Module1,
			Ports::Crio::SolenoidBreakout3,
			Ports::Crio::SolenoidBreakout4);
	
	// Misc
	m_winchMotor = new ReversedVictor(
			Ports::Crio::Module1,
			Ports::DigitalSidecar::Pwm8);
	
	/* Repurposing as pin for defense shield */
	m_loaderSolenoid = new Solenoid( 
			Ports::Crio::Module1,
			Ports::Crio::SolenoidBreakout5);

	m_compressor = new Compressor(
			Ports::Crio::Module1,
			Ports::DigitalSidecar::Gpio1,
			Ports::Crio::Module1,
			Ports::DigitalSidecar::Relay1);
	
	m_xboxDrive = new XboxController(
			Ports::Computer::Usb1);
	m_xboxShooter = new XboxController(
			Ports::Computer::Usb2);
	m_joystick = new Joystick(
			Ports::Computer::Usb3);
}

void MainRobot2013DefenseProfile::CreateSubsystems() {
	// Drive
	Drive::TreadPid leftRatePid(0.1, 0.2, 0.0, 1.0f/2.0f, Encoder::kRate, m_leftEncoder, m_leftTread);
	Drive::TreadPid rightRatePid(0.1, 0.2, 0.0, 1.0f/2.0f, Encoder::kRate, m_rightEncoder, m_rightTread);
	Drive::TreadPid leftDistancePid(0.1, 0.0, 0.0, 240.f / 2.0f, Encoder::kDistance, m_leftEncoder, m_leftTread);
	Drive::TreadPid rightDistancePid(0.1, 0.0, 0.0, 240.f / 2.0f, Encoder::kDistance, m_rightEncoder, m_rightTread);
	
	// todo: validate the above.
	
	Drive::DrivePid rateDrivePid(leftRatePid, rightRatePid);
	Drive::DrivePid distanceDrivePid(leftDistancePid, rightDistancePid);
	/*m_drive = new PidSimpleDrive(
		rateDrivePid,
		distanceDrivePid);*/
	m_drive = new Drive::Simple(
			m_leftTread,
			m_rightTread);
	m_transmission = new DriveTransmission::Simple(
			m_leftTransmissionSolenoid,
			m_rightTransmissionSolenoid);

	// Winch
	m_winch = new Winch::Simple(
			m_winchMotor);
	
	// Repurposing as pin for defense shield
	m_loader = new FrisbeeLoader::Piston(m_loaderSolenoid);
	
	// Diagnostics
	m_leftTestEncoder = new TestEncoder(
			m_leftEncoder, 
			"TestEncoder Drive Left");
	m_rightTestEncoder = new TestEncoder(
			m_rightEncoder, 
			"TestEncoder Drive Right");
}

void MainRobot2013DefenseProfile::CreateOI() {
	m_oi = new CompetitionOI(m_xboxDrive, m_xboxShooter, m_joystick);
}

void MainRobot2013DefenseProfile::RobotInit() {
	
	m_compressor->Start();
	SmartDashboard::PutData("Scheduler", Scheduler::GetInstance());
	/*
	autoChooser = new SendableChooser();
	autoChooser->AddObject("Shoot 3", new AutonomousCommand::FireNFrisbees(m_loader, m_shooter, 3));
	autoChooser->AddDefault("Do nothing", new AutonomousCommand::DoNothing());
	SmartDashboard::PutData("Autonomous Program:", autoChooser);*/
}

void MainRobot2013DefenseProfile::AutonomousInit() {
	/*autoCommand = (Command *)(autoChooser->GetSelected());
	autoCommand->Start();*/
	// empty
	m_compressor->Start();
}

void MainRobot2013DefenseProfile::TeleopInit() {
	m_drive->SetDefaultCommand(new DriveCommand::ArcadeDrive(
			m_drive, 
			m_oi->ArcadeMagAxis, 
			m_oi->ArcadeRotateAxis));
	m_leftTestEncoder->SetDefaultCommand(new TestEncoderCommand(
			m_leftTestEncoder,
			"Drive Left Encoder"));
	m_rightTestEncoder->SetDefaultCommand(new TestEncoderCommand(
			m_rightTestEncoder,
			"Drive Right Encoder"));
	m_oi->DriveStraightButton->WhileHeld(new DriveCommand::TravelStraightManual(
			m_drive,
			m_oi->DriveStraightAxis));
	m_oi->ToggleTransmissionButton->WhenPressed(new DriveCommand::ToggleTransmission(
			m_transmission));
	
	// Repurposing as pin for defense shield
	m_oi->LoadFrisbeeButton->WhenPressed(new ShooterCommand::LoadFrisbee(m_loader));
	
	m_oi->ControlWinchButton->WhileHeld(new WinchCommand::SetSpeed(m_winch, 1.0));
	m_oi->ControlWinchBackButton->WhileHeld(new WinchCommand::SetSpeed(m_winch, -1.0));
	
	m_compressor->Start();
}
