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
	m_compressor = new Compressor(
			Ports::Crio::Module1,
			Ports::DigitalSidecar::Gpio1,
			Ports::Crio::Module1,
			Ports::DigitalSidecar::Relay1);
	
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
	
	// Shooter
	m_shooterBack = new ReversedJaguar(
			Ports::Crio::Module1,
			Ports::DigitalSidecar::Pwm5);
	m_shooterMiddle = new ReversedJaguar(
			Ports::Crio::Module1,
			Ports::DigitalSidecar::Pwm6);
	m_shooterFront = new Jaguar(
			Ports::Crio::Module1,
			Ports::DigitalSidecar::Pwm7);
	m_shooterEncoder = new Encoder(
			Ports::Crio::Module1,
			Ports::DigitalSidecar::Gpio10,
			Ports::Crio::Module1,
			Ports::DigitalSidecar::Gpio11);
	m_shooterEncoder->SetDistancePerPulse(0.2055678);
	m_shooterEncoder->Start();
		
	// Turret
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
			Ports::DigitalSidecar::Gpio6,
			Ports::Crio::Module1,
			Ports::DigitalSidecar::Gpio7);
	m_horizontalTurretEncoder->SetDistancePerPulse(0.1);
	m_horizontalTurretEncoder->Start();
	m_verticalTurretMotor = new Victor(
			Ports::Crio::Module1,
			Ports::DigitalSidecar::Pwm9);
	m_verticalTurretEncoder = new Encoder(
			Ports::Crio::Module1,
			Ports::DigitalSidecar::Gpio12,
			Ports::Crio::Module1,
			Ports::DigitalSidecar::Gpio13);
	//m_verticalTurretEncoder->SetDistancePerPulse(0.024);
	m_verticalTurretEncoder->Start();
	
	m_turretTopSwitch = new DigitalInput(
			Ports::Crio::Module1,
			Ports::DigitalSidecar::Gpio12);
	m_turretBottomSwitch = new DigitalInput(
			Ports::Crio::Module1,
			Ports::DigitalSidecar::Gpio13);
	
	m_winchMotor = new Victor(
			Ports::Crio::Module1,
			Ports::DigitalSidecar::Pwm10);
	
	// On the solenoid...
	// hole 4 goes nearest the piston head
	// hole 2 goes furthest
	m_loaderSolenoid = new Solenoid(
			Ports::Crio::Module1,
			Ports::Crio::SolenoidBreakout5);
	
	m_xboxDrive = new XboxController(
			Ports::Computer::Usb1);
	m_xboxShooter = new XboxController(
			Ports::Computer::Usb2);
	m_joystick = new Joystick(
			Ports::Computer::Usb3);
	
	
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
			m_horizontalTurretMotor,
			"FrisbeeTurret::Simple horizontal");
	m_verticalTurret = new FrisbeeTurret::Guarded(
			m_verticalTurretMotor, 
			m_turretLeftSwitch,
			m_turretRightSwitch,
			"FrisbeeTurret::Simple vertical");
	m_horizontalTurretPosition = new TurretPosition::EncoderAngle(
			m_horizontalTurretEncoder,
			0,
			"TurretPosition::EncoderAngle Horizontal");
	m_verticalTurretPosition = new TurretPosition::EncoderAngle(
			m_verticalTurretEncoder,
			30,
			"TurretPosition::EncoderAngle Vertical");

	m_aimer = NULL; //new FrisbeeAimer::VisionTables();

	m_horizontalTurretTestEncoder = new TestEncoder(m_horizontalTurretEncoder, "Turret Horizontal Test");
	m_verticalTurretTestEncoder = new TestEncoder(m_verticalTurretEncoder, "Turret Vertical Test");
	
	m_shooterTestEncoder = new TestEncoder(m_shooterEncoder, "Shooter Test");
	
	m_shooter = new FrisbeeShooter::ThreeWheel(
		m_shooterFront,
		m_shooterMiddle,
		m_shooterBack);	
	
	m_winch = new Winch::Simple(m_winchMotor);
}

void AugmentedTerminatorRobotProfile::CreateOI() {
	m_oi = new CompetitionOI(m_xboxDrive, m_xboxShooter, m_joystick);
}

void AugmentedTerminatorRobotProfile::RobotInit() {
	m_drive->SetDefaultCommand(new DriveCommand::TankDrive(
			m_drive, 
			m_oi->TankLeftAxis,
			m_oi->TankRightAxis));
	m_drive->SetDefaultCommand(new DriveCommand::ArcadeDrive(
			m_drive, 
			m_oi->ArcadeMagAxis, 
			m_oi->ArcadeRotateAxis));
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
	m_verticalTurret->SetDefaultCommand(new ShooterCommand::ManuallyControlTurret(
			m_verticalTurret, 
			m_oi->LiftTurretAxis,
			"ManuallyControlTurretCommand_Vertical"));
	m_verticalTurretTestEncoder->SetDefaultCommand(new TestEncoderCommand(
			m_verticalTurretTestEncoder, 
			"turret vertical Encoder"));
	
	m_shooterTestEncoder->SetDefaultCommand(new TestEncoderCommand(
			m_shooterTestEncoder,
			"shooter Encoder"));
	
	m_oi->FireFrisbeeButton->WhileHeld(new ShooterCommand::FireFrisbeeWithAdjustableSpeed(
			m_shooter, 
			m_oi->ShooterSpeedAxis));
	m_oi->LoadFrisbeeButton->WhileHeld(new ShooterCommand::LoadFrisbee(
			m_loader));
	m_oi->SetTurretHomeButton->WhileHeld(new ShooterCommand::SetTurretHome(
			m_horizontalTurretPosition,
			"ShooterCommand::SetTurretHome Horizontal"));
	m_oi->GotoTurretHomeButton->WhileHeld(new ShooterCommand::MoveTurretHome(
			m_horizontalTurret,
			m_horizontalTurretPosition,
			"ShooterCommand::MoveTurretHome Horizontal"));
	
	m_oi->ControlWinchButton->WhileHeld(new WinchCommand::SetSpeed(m_winch, 1.0));
	
	SmartDashboard::PutData("Scheduler", Scheduler::GetInstance());
}

void AugmentedTerminatorRobotProfile::AutonomousInit() {
	// empty
}

void AugmentedTerminatorRobotProfile::TeleopInit() {
	m_compressor->Start();
	m_horizontalTurretEncoder->Reset();
	m_verticalTurretEncoder->Reset();
	m_leftEncoder->Reset();
	m_rightEncoder->Reset();
}

void AugmentedTerminatorRobotProfile::TeleopPeriodic() {
	Scheduler::GetInstance()->Run();
}


void AugmentedTerminatorRobotProfile::DisabledInit() {
	Scheduler::GetInstance()->RemoveAll();
}
