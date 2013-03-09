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
	
	// Transmission
	m_rightTransmissionSolenoid = new DoubleSolenoid(
		Ports::Crio::Module1,
		Ports::Crio::SolenoidBreakout1,
		Ports::Crio::SolenoidBreakout2);
	m_leftTransmissionSolenoid = new DoubleSolenoid(
		Ports::Crio::Module1,
		Ports::Crio::SolenoidBreakout3,
		Ports::Crio::SolenoidBreakout4);
	
	// Turret
	m_turretVertical = new Victor(
		Ports::Crio::Module1,
		Ports::DigitalSidecar::Pwm3);
	m_turretHorizontal = new Victor(
		Ports::Crio::Module1,
		Ports::DigitalSidecar::Pwm4);
	m_turretRightSwitch = new DigitalInput(
		Ports::Crio::Module1,
		Ports::DigitalSidecar::Gpio8);
	m_turretLeftSwitch = new DigitalInput(
		Ports::Crio::Module1,
		Ports::DigitalSidecar::Gpio9);
	m_turretTopSwitch = new DigitalInput(
		Ports::Crio::Module1,
		Ports::DigitalSidecar::Gpio10);
	m_turretBottomSwitch = new DigitalInput(
		Ports::Crio::Module1,
		Ports::DigitalSidecar::Gpio11);
	
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
	m_shooterEncoder = new Encoder(
		Ports::Crio::Module1,
		Ports::DigitalSidecar::Gpio6,
		Ports::Crio::Module1,
		Ports::DigitalSidecar::Gpio7);
	
	// Misc
	m_winch = new Victor(
			Ports::Crio::Module1,
			Ports::DigitalSidecar::Pwm8);
	
	m_xbox = new XboxController(
			Ports::Computer::Usb1);
	m_leftStick = new Joystick(
			Ports::Computer::Usb2);
	m_rightStick = new Joystick(
			Ports::Computer::Usb3);
	
	m_compressor = new Compressor(
			Ports::Crio::Module1,
			Ports::DigitalSidecar::Gpio1,
			Ports::Crio::Module1,
			Ports::DigitalSidecar::Relay1);
}

void MainRobot2013Profile::CreateSubsystems() {
	IntegratedPid leftRatePid(0.1, 0.2, 0.0, 1.0f/2.0f, Encoder::kRate, m_leftEncoder, m_leftTread);
	IntegratedPid rightRatePid(0.1, 0.2, 0.0, 1.0f/2.0f, Encoder::kRate, m_rightEncoder, m_rightTread);
	IntegratedPid leftDistancePid(0.1, 0.0, 0.0, 240.f / 2.0f, Encoder::kDistance, m_leftEncoder, m_leftTread);
	IntegratedPid rightDistancePid(0.1, 0.0, 0.0, 240.f / 2.0f, Encoder::kDistance, m_rightEncoder, m_rightTread);
	
	// todo: validate the above.
	
	DrivePid rateDrivePid(leftRatePid, rightRatePid);
	DrivePid distanceDrivePid(leftDistancePid, rightDistancePid);
	/*m_drive = new PidSimpleDrive(
		rateDrivePid,
		distanceDrivePid);*/
	m_drive = new SimpleDrive(
			m_leftTread,
			m_rightTread);
	m_loader = new PlaceholderFrisbeeLoader();
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
			m_turretVertical);*/
	m_horizontalTurret = new SimpleAxisFrisbeeTurret(m_turretHorizontal);
	m_verticalTurret = new SimpleAxisFrisbeeTurret(m_turretVertical);
	m_shooter = new ThreeWheelShooter(
		m_shooterFront,
		m_shooterMiddle,
		m_shooterBack);
	m_transmission = new SimpleDriveTransmission(
		m_leftTransmissionSolenoid,
		m_rightTransmissionSolenoid);
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
	m_oi->ToggleTransmissionButton->WhenPressed(new ToggleTransmissionCommand(
			m_transmission));
	
	/*m_turret->SetDefaultCommand(new ManuallyControlTurretCommand(
			m_turret,
			m_oi->RotateTurretAxis,
			m_oi->LiftTurretAxis));*/
	m_oi->FireFrisbeeButton->WhileHeld(new FireFrisbeeCommand( 
			m_shooter));
	
	m_compressor->Start();
	
	SmartDashboard::PutData(m_horizontalTurret);
	SmartDashboard::PutData(m_verticalTurret);
	SmartDashboard::PutData(m_drive);
	SmartDashboard::PutData(m_shooter);
	
	/*m_turret->SetDefaultCommand(new AimTurretCommand(
			m_aimer, 
			m_turret, 
			Tracking::ClosestOffset, 
			3.0));*/
	
	// Will stop if the shooter is within 3 degrees of the centerpoint of the target
}
