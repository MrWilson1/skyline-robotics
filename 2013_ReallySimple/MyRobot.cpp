#include "WPILib.h"
#include "xbox.h"
/**
 * This is a demo program showing the use of the RobotBase class.
 * The SimpleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 */ 
class RobotDemo : public SimpleRobot
{
	RobotDrive *drive;
	XboxController *xbox;

public:
	RobotDemo(void)
	{
		drive = new RobotDrive(1, 2, 3, 4);
		xbox = new XboxController(1);
		drive->SetExpiration(0.1);
	}

	/**
	 * Drive left & right motors for 2 seconds then stop
	 */
	void Autonomous(void)
	{
		Wait(4);
	}

	/**
	 * Runs the motors with arcade steering. 
	 */
	void OperatorControl(void)
	{
		drive->SetSafetyEnabled(true);
		while (IsOperatorControl())
		{
			float leftY = Cutoff(xbox->GetAxis(xbox->LeftY));
			float rightY = Cutoff(xbox->GetAxis(xbox->RightY));
			drive->TankDrive(leftY, rightY);
			Wait(0.005);				// wait for a motor update time
		}
	}
	
	float Cutoff(float num)
	{
		if ((-0.1 <= num) and (num <= 0.1)) {
			num = 0;
		}
		return num;
	}
	
	/**
	 * Runs during test mode
	 */
	void Test() {
		
	}
};

START_ROBOT_CLASS(RobotDemo);

