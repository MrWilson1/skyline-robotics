#include "MainRobot2013Profile.h"

MainRobot2013Profile::MainRobot2013Profile() :
		BaseRobotProfile("MainRobot2013Profile") {
	CreateBasicHardwareObjects();
	CreateSubsystems();
	CreateOI();
}

MainRobot2013Profile::~MainRobot2013Profile() {
	// empty
}

void MainRobot2013Profile::CreateBasicHardwareObjects() {
	// Drive
	m_leftFront = new Talon(
		Ports::Crio::Module1,
		Ports::DigitalSidecar::Pwm1);
	m_leftBack = new Talon(
		Ports::Crio::Module1,
		Ports::DigitalSidecar::Pwm2);
	m_rightFront = new Talon(
		Ports::Crio::Module1,
		Ports::DigitalSidecar::Pwm3);
	m_rightBack = new Talon(
		Ports::Crio::Module1,
		Ports::DigitalSidecar::Pwm4);
	m_leftEncoder = new Encoder(
		Ports::Crio::Module1,
		Ports::DigitalSidecar::Gpio1,
		Ports::Crio::Module1,
		Ports::DigitalSidecar::Gpio2);
	m_rightEncoder = new Encoder(
		Ports::Crio::Module1,
		Ports::DigitalSidecar::Gpio3,
		Ports::Crio::Module1,
		Ports::DigitalSidecar::Gpio4);
	
	// Turret
	m_turretHorizontal = new Talon(
		Ports::Crio::Module1,
		Ports::DigitalSidecar::Pwm5);
	m_turretVertical = new Talon(
		Ports::Crio::Module1,
		Ports::DigitalSidecar::Pwm6);
	m_turretHorizontalEncoder = new Encoder(
		Ports::Crio::Module1,
		Ports::DigitalSidecar::Gpio5,
		Ports::Crio::Module1,
		Ports::DigitalSidecar::Gpio6);
	m_turretVerticalEncoder = new Encoder(
		Ports::Crio::Module1,
		Ports::DigitalSidecar::Gpio7,
		Ports::Crio::Module1,
		Ports::DigitalSidecar::Gpio8);
	
	// Shooter
	m_shooterFront = new Talon(
		Ports::Crio::Module1,
		Ports::DigitalSidecar::Pwm7);
	m_shooterMiddle = new Talon(
		Ports::Crio::Module1,
		Ports::DigitalSidecar::Pwm8);
	m_shooterLast = new Talon(
		Ports::Crio::Module1,
		Ports::DigitalSidecar::Pwm9);
	
	m_xbox = new XboxController(
			Ports::Computer::Usb1);
	m_leftStick = new Joystick(
			Ports::Computer::Usb2);
	m_rightStick = new Joystick(
			Ports::Computer::Usb3);
}

void MainRobot2013Profile::CreateSubsystems() {
	m_drive = new PidSimpleDrive(
		m_leftFront, 
		m_leftBack, 
		m_rightFront,
		m_rightBack,
		m_leftEncoder,
		m_rightEncoder,
		1.0f / 4134.0f,
		1.0f / 4054.0f,
		240.f / 4134.0f,
		240.f / 4054.0f);
	m_loader = new PlaceholderFrisbeeLoader();
	m_aimer = new VisionTablesFrisbeeAimer();
	m_turret = new PidFrisbeeTurret(
		m_turretHorizontal,
		m_turretVertical,
		m_turretHorizontalEncoder,
		m_turretVerticalEncoder);
	m_shooter = new ThreeWheelShooter(
		m_shooterFront,
		m_shooterMiddle,
		m_shooterLast);
}

void MainRobot2013Profile::CreateOI() {
	m_oi = new XboxTwoJoysticksOI(m_xbox, m_leftStick, m_rightStick);
}

void MainRobot2013Profile::RobotInit() {
	// empty
}

void MainRobot2013Profile::AutonomousInit() {
	// empty
}

void MainRobot2013Profile::TeleopInit() {
	m_drive->SetDefaultCommand(new TankDriveCommand(
			m_drive, 
			m_oi->TankLeftAxis,
			m_oi->TankRightAxis));
	m_oi->DriveStraightButton->WhileHeld(new TravelStraightManualCommand(
			m_drive,
			m_oi->DriveStraightAxis));
	
	m_turret->SetDefaultCommand(new ManuallyAdjustTurretCommand(
			m_turret,
			m_oi->RotateTurretAxis,
			m_oi->LiftTurretAxis,
			3.0));
	m_oi->FireFrisbeeButton->WhenPressed(new FireFrisbeeCommand( 
			m_shooter));
	
	/*m_turret->SetDefaultCommand(new AimTurretCommand(
			m_aimer, 
			m_turret, 
			Tracking::ClosestOffset, 
			3.0));*/
	
	// Will stop if the shooter is within 3 degrees of the centerpoint of the target
}
