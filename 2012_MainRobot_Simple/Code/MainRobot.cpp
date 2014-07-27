#include "MainRobot.h"

MainRobot::MainRobot() {
	InitializeHardware();
	InitializeSoftware();
}

void MainRobot::RobotInit()
{	
}

// InitializeHardware: Objects for interacting with hardware are initialized here
void MainRobot::InitializeHardware()
{
	m_drive 			= new RobotDrive(Ports::DigitalSidecar::Pwm8, 
							 	 	 	 Ports::DigitalSidecar::Pwm10);
	driveController 	= new XboxController(Ports::Computer::Usb1);
	topLeftShooter 		= new Jaguar(Ports::DigitalSidecar::Pwm1);
	topRightShooter 	= new Jaguar(Ports::DigitalSidecar::Pwm2);
	bottomLeftShooter 	= new Jaguar(Ports::DigitalSidecar::Pwm3);
	bottomRightShooter 	= new Jaguar(Ports::DigitalSidecar::Pwm4);
	elevator 			= new Jaguar(Ports::DigitalSidecar::Pwm5);
	//arm 				= new Jaguar(Ports::DigitalSidecar::Pwm6);
}

// InitializeSoftware: Initialize subsystems
void MainRobot::InitializeSoftware()
{
	shooter = new Shooter(topLeftShooter, topRightShooter, bottomLeftShooter, bottomRightShooter);
}

void MainRobot::Autonomous()
{
}

void MainRobot::OperatorControl()
{
	m_drive->SetSafetyEnabled(true);
	while (IsOperatorControl()) {
		// (for the two code lines below) The minus in front makes the robot drive in the
		// direction of the collector (when joysticks are forward)
		float leftY = -Cutoff(driveController->GetAxis(driveController->LeftY));
		float rightY = -Cutoff(driveController->GetAxis(driveController->RightY));
		
		m_drive->TankDrive(leftY, rightY);
		
		float loaderY = Cutoff(driveController->GetYDirDpad());
		if (loaderY > 0.4) {
			// Move loader upwards (take in ball)
			elevator->Set(1);
		} else if (loaderY < -0.4) {
			// Move loader downwards (put out ball)
			elevator->Set(-1);
		} else {
			elevator->Set(0.0);
		}
		
		float triggerY = Cutoff(driveController->GetTriggerAxis());
		if (triggerY < -0.4) {
			shooter->SetSpeed(1);
		} else {
			shooter->SetSpeed(0);
		}
	}
	Wait(0.005); // wait for a motor update time
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
