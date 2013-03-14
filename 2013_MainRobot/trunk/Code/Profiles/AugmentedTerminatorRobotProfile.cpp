#include "AugmentedTerminatorRobotProfile.h"

AugmentedTerminatorRobotProfile::AugmentedTerminatorRobotProfile() :
		BaseRobotProfile("AugmentedTerminatorProfile") {
	CreateBasicHardwareObjects();
	CreateSubsystems();
	CreateOI();
}

AugmentedTerminatorRobotProfile::~AugmentedTerminatorRobotProfile() {
	// empty
}

void AugmentedTerminatorRobotProfile::CreateBasicHardwareObjects() {
	// Drive
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
	m_leftTread = new Tread(
			m_leftFrontMotor,
			m_leftBackMotor);
	m_rightTread = new Tread(
			m_rightFrontMotor,
			m_rightBackMotor);
	m_rightTread->SetDirection(Tread::kReverse);
	
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
		
	// Turret
	/*
	m_verticalTurretMotor = new Victor(
		Ports::Crio::Module1,
		Ports::DigitalSidecar::Pwm9);
	m_turretTopSwitch = new DigitalInput(
		Ports::Crio::Module1,
		Ports::DigitalSidecar::Gpio12);
	m_turretBottomSwitch = new DigitalInput(
		Ports::Crio::Module1,
		Ports::DigitalSidecar::Gpio13);
	*/
	m_horizontalTurretMotor = new Victor(
		Ports::Crio::Module1,
		Ports::DigitalSidecar::Pwm8);
	m_turretRightSwitch = new DigitalInput(
		Ports::Crio::Module1,
		Ports::DigitalSidecar::Gpio8);
	m_turretLeftSwitch = new DigitalInput(
		Ports::Crio::Module1,
		Ports::DigitalSidecar::Gpio9);
	m_horizontalTurretEncoder = new Encoder(
			Ports::Crio::Module1,
			Ports::DigitalSidecar::Gpio10,
			Ports::Crio::Module1,
			Ports::DigitalSidecar::Gpio11);
	m_horizontalTurretEncoder->Start();
			
		
	// Shooter
	
	m_shooterBack = new Victor(
		Ports::Crio::Module1,
		Ports::DigitalSidecar::Pwm5);
	m_shooterMiddle = new Victor(
		Ports::Crio::Module1,
		Ports::DigitalSidecar::Pwm6);
	m_shooterFront = new Victor(
		Ports::Crio::Module1,
		Ports::DigitalSidecar::Pwm7);
	
	
	/*
	m_shooterEncoder = new Encoder(
		Ports::Crio::Module1,
		Ports::DigitalSidecar::Gpio1,
		Ports::Crio::Module1,
		Ports::DigitalSidecar::Gpio2);
	*/
	
	// On the solenoid...
	// hole 4 goes nearest the piston head
	// hole 2 goes furthest
	m_loaderSolenoid = new Solenoid(
			Ports::Crio::Module1,
			Ports::Crio::SolenoidBreakout5);
	
	m_xbox = new XboxController(
			Ports::Computer::Usb1);
	m_joystick = new Joystick(
			Ports::Computer::Usb2);
	
	m_compressor = new Compressor(
			Ports::Crio::Module1,
			Ports::DigitalSidecar::Gpio1,
			Ports::Crio::Module1,
			Ports::DigitalSidecar::Relay1);
}

void AugmentedTerminatorRobotProfile::CreateSubsystems() {
	// These currently have no effect on the robot whatsoever.
	// 4134 and 4054
	Drive::TreadPid leftRatePid(0.1, 0.2, 0.0, 3000.0f, Encoder::kRate, m_leftEncoder, m_leftTread);
	Drive::TreadPid rightRatePid(0.1, 0.2, 0.0, 3000.0f, Encoder::kRate, m_rightEncoder, m_rightTread);
	Drive::TreadPid leftDistancePid(0.015, 0.0, 0.0, 4134.0f, Encoder::kDistance, m_leftEncoder, m_leftTread);
	Drive::TreadPid rightDistancePid(0.015, 0.0, 0.0, 4054.0f, Encoder::kDistance, m_rightEncoder, m_rightTread);
	
	Drive::DrivePid rateDrivePid(leftRatePid, rightRatePid);
	Drive::DrivePid distancePid(leftDistancePid, rightDistancePid);
	
	//m_drive = new PidSimpleDrive(rateDrive::DrivePid, distanceDrive::DrivePid, 34.0);
	m_drive = new Drive::Simple(m_leftTread, m_rightTread);
	m_leftTestEncoder = new TestEncoder(m_leftEncoder, "Left Encoder Test");
	m_rightTestEncoder = new TestEncoder(m_rightEncoder, "Right Encoder Test");
	
	m_loader = new FrisbeeLoader::Piston(m_loaderSolenoid);
	
	m_horizontalTurret = new FrisbeeTurret::Simple(
			m_horizontalTurretMotor);
	/*
			m_turretLeftSwitch,
			m_turretRightSwitch);*/
	/*m_verticalTurret = new SimpleAxisFrisbeeTurret(
			m_verticalTurretMotor);
	*/
	//m_aimer = new FrisbeeAimer::VisionTables();
	//m_aimerTest = new FrisbeeAimer::Test("VisionTest");
	
	m_horizontalTurretTestEncoder = new TestEncoder(m_horizontalTurretEncoder, "Turret Horizontal Test");
	
	m_shooter = new FrisbeeShooter::ThreeWheel(
		m_shooterFront,
		m_shooterMiddle,
		m_shooterBack);
	
}

void AugmentedTerminatorRobotProfile::CreateOI() {
	m_oi = new XboxSingleJoystickOI(m_xbox, m_joystick);
}

void AugmentedTerminatorRobotProfile::RobotInit() {
	// empty
}

void AugmentedTerminatorRobotProfile::AutonomousInit() {
	// empty
}

void AugmentedTerminatorRobotProfile::TeleopInit() {
	m_drive->SetDefaultCommand(new DriveCommand::TankDrive(
			m_drive, 
			m_oi->TankLeftAxis,
			m_oi->TankRightAxis));
	m_leftTestEncoder->SetDefaultCommand(new TestEncoderCommand(
			m_leftTestEncoder,
			"left Encoder"));
	m_rightTestEncoder->SetDefaultCommand(new TestEncoderCommand(
			m_rightTestEncoder,
			"right Encoder"));
	m_oi->DriveStraightButton->WhileHeld(new DriveCommand::TravelStraightManual(
			m_drive,
			m_oi->DriveStraightAxis));
	
	
	m_horizontalTurret->SetDefaultCommand(new ShooterCommand::ManuallyControlTurret(
			m_horizontalTurret, 
			m_oi->RotateTurretAxis,
			"ManuallyControlTurretCommand_Horizontal"));
	m_horizontalTurretTestEncoder->SetDefaultCommand(new TestEncoderCommand(
			m_horizontalTurretTestEncoder, 
			"turret horizontal Encoder"));
	/*m_verticalTurret->SetDefaultCommand(new ManuallyControlTurretCommand(
			m_verticalTurret, 
			m_oi->RotateTurretAxis,
			"ManuallyControlTurretCommand_Vertical"));
	*/
	
	m_oi->LoadFrisbeeButton->WhenPressed(new ShooterCommand::LoadFrisbee(m_loader));
	m_oi->FireFrisbeeButton->WhileHeld(new ShooterCommand::FireFrisbee(m_shooter));
	/*SmartDashboard::PutData(
			"SmartDashboardFireFrisbee", 
			new ShooterCommand::SmartDashboardFireFrisbee(m_shooter));
	*/
	m_compressor->Start();
	
	//SmartDashboard::PutData("Scheduler", Scheduler::GetInstance());
	
	// Will stop if the shooter is within 3 degrees of the centerpoint of the target
}

void AugmentedTerminatorRobotProfile::TeleopPeriodic() {
	Scheduler::GetInstance()->Run();
	//m_aimerTest->Run();
}
