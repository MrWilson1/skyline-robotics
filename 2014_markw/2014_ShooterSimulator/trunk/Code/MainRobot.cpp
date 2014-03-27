#include "MainRobot.h"

#define XBOX 1
#define JOYSTICKS 2 // Joystick controlling not implemented
#define TANK 1
#define ARCADE 2
#define ARCADE2 3

int CONTROLLER = XBOX;
int DRIVING = ARCADE2;

MainRobot::MainRobot() {
	InitializeHardware();
	InitializeSoftware();
}

void MainRobot::WatchdogWait(double time) {
	Timer* timer = new Timer();
	timer->Start();
	while (true) {
		RobotBase::getInstance().GetWatchdog().Feed();
		if (timer->Get() >= time) {
			break;
		}
		Wait(.05);
	}
}

void MainRobot::RobotInit() {
	
}

// InitializeHardware: Objects for interacting with hardware are initialized here
void MainRobot::InitializeHardware()
{
/*	m_drive = new RobotDrive(Ports::DigitalSidecar::Pwm8, 
							 Ports::DigitalSidecar::Pwm7, 
							 Ports::DigitalSidecar::Pwm10, 
							 Ports::DigitalSidecar::Pwm9);
	m_drive->SetSafetyEnabled(true);
*/
	if (CONTROLLER == XBOX) {
		driveController = new XboxController(Ports::Computer::Usb1);
		shootController = new XboxController(Ports::Computer::Usb2);
	} else if (CONTROLLER == JOYSTICKS) {
		m_leftStick = new Joystick(Ports::Computer::Usb1);
		m_rightStick = new Joystick(Ports::Computer::Usb2);
	}

/*	m_compressor = new Compressor(Ports::DigitalSidecar::Gpio1,
								  Ports::DigitalSidecar::Relay1);
	m_compressor->Start();
	
	m_solenoid1 = new Solenoid(Ports::Crio::SolenoidBreakout1);
	m_solenoid2 = new Solenoid(Ports::Crio::SolenoidBreakout2);
	m_solenoid3 = new Solenoid(Ports::Crio::SolenoidBreakout3);
	m_solenoid4 = new Solenoid(Ports::Crio::SolenoidBreakout4);
	
	//m_clawMotor = new Victor(Ports::DigitalSidecar::Pwm5);
	
	m_collectorMotor = new Victor(Ports::DigitalSidecar::Pwm1);
	m_collectorMotor->SetSafetyEnabled(true);
*/
	m_LeftshooterMotors = new Talon(Ports::Crio::Module2,Ports::DigitalSidecar::Pwm4);  
	m_RightshooterMotors = new Talon(Ports::Crio::Module2,Ports::DigitalSidecar::Pwm5);// This one Talon object powers
																// all four shooter motors
//	m_LeftshooterMotors->SetSafetyEnabled(false);
//	m_RightshooterMotors->SetSafetyEnabled(false);
//	
//	m_pistonLimitSwitch = new DigitalInput(Ports::DigitalSidecar::Gpio11);
//	m_shooterLimitSwitch = new DigitalInput(Ports::DigitalSidecar::Gpio12);
}

// InitializeSoftware: Initialize subsystems
void MainRobot::InitializeSoftware()
{
	//m_claw = new Claw(m_clawMotor);
//	m_collector = new Collector(m_collectorMotor, m_solenoid1, m_solenoid2,
//			m_solenoid3, m_solenoid4, m_compressor, m_pistonLimitSwitch);
	m_shooter = new Shooter(m_LeftshooterMotors, m_RightshooterMotors, m_shooterLimitSwitch, m_collector);
	//netTable = NetworkTable::GetTable("VisionTargetInfo");
	//netTable->PutNumber("Driving", DRIVING);
	m_timer = new Timer();
}


void MainRobot::Autonomous()
{
}

bool isShooting = false;
//int nextImageCheck = 0;
void MainRobot::OperatorControl()
{
	//m_drive->SetSafetyEnabled(true);
	
	m_timer->Stop();
	m_timer->Reset();
	m_timer->Start();
		
//	int operatorControlLifetime = 0;
	//AxisCamera &camera = AxisCamera::GetInstance("10.29.76.11");
	
	while (IsOperatorControl()) {

		if (CONTROLLER == XBOX) {
			// SHOOTING
			// ----------------------------------------------------------------------
			// The Set() values below (-.15 & .15) indicate the power
			// to the motor (15%) and are very important to understand
			// before changing. It controls the speed of the shooter
			// arm while the button is pressed. If the values are too high,
			// you run the risk of wrapping the arm into the robot
			// (or around the robot).
			
			if (shootController->GetAButton()) {
				m_shooter->Set(1);
			} else if (shootController->GetBButton()) {
				m_shooter->Set(-1);
			} else {
				m_shooter->Stop();
			}
						
			float trigger = shootController->GetTriggerAxis();
			if (trigger <= -0.4){
				if (!isShooting) {
//					m_compressor->Stop();
					isShooting = true;
					m_shooter->ShootWithArm();
//					m_compressor->Start();
				}
			} else {
				isShooting = false;
			}
		}

//		SmartDashboard::PutBoolean("shooter limit switch",m_shooter->GetLimitSwitch());
		Wait(0.005); // wait for a motor update time
	}
}

void MainRobot::Test()
{
	
}

float MainRobot::Cutoff(float num)
{
	if ((-0.1 <= num) && (num <= 0.1)) {
		num = 0;
	}
	return num;
}

START_ROBOT_CLASS(MainRobot);
