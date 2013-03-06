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
	/*m_turretVertical = new Victor(
		Ports::Crio::Module1,
		Ports::DigitalSidecar::Pwm5);*/
	m_turretHorizontal = new Victor(
		Ports::Crio::Module1,
		Ports::DigitalSidecar::Pwm6);
	m_turretRightSwitch = new DigitalInput(
		Ports::Crio::Module1,
		Ports::DigitalSidecar::Gpio8);
	m_turretLeftSwitch = new DigitalInput(
		Ports::Crio::Module1,
		Ports::DigitalSidecar::Gpio9);
	/*
	m_turretTopSwitch = new DigitalInput(
		Ports::Crio::Module1,
		Ports::DigitalSidecar::Gpio10);
	m_turretBottomSwitch = new DigitalInput(
		Ports::Crio::Module1,
		Ports::DigitalSidecar::Gpio11);*/
	
	// Shooter
	/*m_shooterBack = new Victor(
		Ports::Crio::Module1,
		Ports::DigitalSidecar::Pwm5);
	m_shooterMiddle = new Victor(
		Ports::Crio::Module1,
		Ports::DigitalSidecar::Pwm6);
	m_shooterFront = new Victor(
		Ports::Crio::Module1,
		Ports::DigitalSidecar::Pwm7);
	m_shooterEncoder = new Encoder(
		Ports::Crio::Module1,
		Ports::DigitalSidecar::Gpio1,
		Ports::Crio::Module1,
		Ports::DigitalSidecar::Gpio2);*/
	
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
	IntegratedPid leftRatePid(0.1, 0.2, 0.0, 3000.0f, Encoder::kRate, m_leftEncoder, m_leftTread);
	IntegratedPid rightRatePid(0.1, 0.2, 0.0, 3000.0f, Encoder::kRate, m_rightEncoder, m_rightTread);
	IntegratedPid leftDistancePid(0.015, 0.0, 0.0, 4134.0f, Encoder::kDistance, m_leftEncoder, m_leftTread);
	IntegratedPid rightDistancePid(0.015, 0.0, 0.0, 4054.0f, Encoder::kDistance, m_rightEncoder, m_rightTread);
	
	DrivePid rateDrivePid(leftRatePid, rightRatePid);
	DrivePid distanceDrivePid(leftDistancePid, rightDistancePid);
	
	//m_drive = new PidSimpleDrive(rateDrivePid, distanceDrivePid, 34.0);
	m_drive = new SimpleDrive(m_leftTread, m_rightTread);
	m_leftTestEncoder = new TestEncoder(m_leftEncoder, "Left Encoder Test");
	m_rightTestEncoder = new TestEncoder(m_rightEncoder, "Right Encoder Test");
	
	m_loader = new PistonFrisbeeLoader(m_loaderSolenoid);
	//m_aimer = new VisionTablesFrisbeeAimer();
	/*m_turret = new GuardedFrisbeeTurret(
			m_turretHorizontal,
			m_turretVertical,
			m_turretLeftSwitch,
			m_turretRightSwitch,
			m_turretTopSwitch,
			m_turretBottomSwitch);
	*/
	/*m_turret = new SimpleFrisbeeTurret(
			m_turretHorizontal, 
			m_turretVertical);
	m_shooter = new ThreeWheelShooter(
		m_shooterFront,
		m_shooterMiddle,
		m_shooterBack);*/
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
	
	/**m_turret->SetDefaultCommand(new ManuallyControlTurretCommand(
			m_turret,
			m_oi->RotateTurretAxis,
			m_oi->LiftTurretAxis));*/
	/*m_oi->FireFrisbeeButton->WhileHeld(new LoadAndFireFrisbeeCommand(
			m_loader,
			m_shooter));*/
	//m_oi->LoadFrisbeeButton->WhenPressed(new LoadFrisbeeCommand(m_loader));
	
	m_compressor->Start();
	
	//SmartDashboard::PutData(m_turret);
	SmartDashboard::PutData(new LoadFrisbeeCommand(m_loader));
	SmartDashboard::PutData(m_drive);
	//SmartDashboard::PutData(m_shooter);
	SmartDashboard::PutData(m_loader);
	
	/*m_turret->SetDefaultCommand(new AimTurretCommand(
			m_aimer, 
			m_turret, 
			Tracking::ClosestOffset, 
			3.0));*/
	
	// Will stop if the shooter is within 3 degrees of the centerpoint of the target
}
