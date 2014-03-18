#include "WPILib.h"

/**
 * This is a demo program showing the use of the RobotBase class.
 * The SimpleRobot class is the base of a robot application that 
 * will automatically call your Autonomous and OperatorControl 
 * methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 */ 
class RobotDemo : public SimpleRobot
{
	RobotDrive myRobot; // robot drive system
	Joystick leftStick; 
	Joystick rightStick;
	Joystick twistStick;
	Jaguar topLeftShooter;
	Jaguar topRightShooter;
	Jaguar bottomLeftShooter;
	Jaguar bottomRightShooter;
	Jaguar elevator;
	Jaguar arm;
	DigitalInput elevatorBottomLimit;
	DigitalInput topLimitSwitch;
	DigitalInput bottomLimitSwitch;

public:
	RobotDemo(void):
		myRobot(8, 7, 10, 9),	// these must be initialized in the same order - Only PWM 8 & 10 Used - same as 2014 robot
		leftStick(1),			// USB 1
		rightStick(2),			// USB 2
		twistStick(3),
		topLeftShooter(5),		// PWM 5
		topRightShooter(6),		// PWM 6
		bottomLeftShooter(1),	// PWM 1
		bottomRightShooter(2),	// PWM 2
		elevator(4),			// PWM 4 - Same as 2014 Main Robot Shooter.  This is a safe port for both
		arm(3),					// PWM 3
		elevatorBottomLimit(1),	// GPIO 1
		topLimitSwitch(2),		// GPIO 2
		bottomLimitSwitch(3)	// GPIO 3
	{
		myRobot.SetExpiration(0.1);
	}

	/**
	 * Drive left & right motors for 2 seconds then stop
	 */
	void Autonomous(void)
	{
		myRobot.SetSafetyEnabled(false);
		myRobot.Drive(-0.5, 0.0); 	// drive forwards half speed
		Wait(2.0); 				//    for 2 seconds
		myRobot.Drive(0.0, 0.0); 	// stop robot
	}

	/**
	 * Runs the motors with arcade steering. 
	 */
	void OperatorControl(void)
	{
		myRobot.SetSafetyEnabled(true);
		while (IsOperatorControl())
		{
			myRobot.TankDrive(-leftStick.GetY(), -rightStick.GetY());
			SetShooterSpeed(ShooterSpeedMap());
			SetElevatorSpeed(rightStick.GetRawButton(6) ? -1 : 0 +
					rightStick.GetRawButton(7) ? 1 : 0);
			
			Wait(0.005);				// wait for a motor update time
		}
	}
	
	float ShooterSpeedMap() {
		if (twistStick.GetRawButton(11)) {
			return 0.3;
		} else if (twistStick.GetRawButton(9)){
			return 0.35;
		} else if (twistStick.GetRawButton(7)) {
			return 0.4;
		} else {
			return 0.0;
		}
	}
	
	void SetShooterSpeed(float speed) {
		float kReductionFactor = 0.9;
		
		float slowSpeed = speed * kReductionFactor;
			
		topLeftShooter.Set(slowSpeed);
		topRightShooter.Set(-1 * slowSpeed);
		bottomLeftShooter.Set(-1 * speed);
		bottomRightShooter.Set(speed);
	}
	
	void SetElevatorSpeed(float speed) {
		elevator.Set(speed);
	}
	
	void SetArmSpeed(float speed) {
		if (topLimitSwitch.Get() && speed > 0) {
			speed = 0;
		} else if (bottomLimitSwitch.Get() && speed < 0) {
			speed = 0;
		}
		arm.Set(speed);
	}
	
	/**
	 * Runs during test mode
	 */
	void Test() {

	}
};

START_ROBOT_CLASS(RobotDemo);

