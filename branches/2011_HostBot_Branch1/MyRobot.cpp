/* **********************************************
 * Skyline High School Robotics Team
 * Spartabots, Team 2976
 * Main code for team robot
 * FRC Robotics competition 2011 - Logomotion
 ***********************************************/

/*-------------------- Recommended Maximum Length of Lines -------------------*/

#include "WPILib.h"
#include "math.h"



/**
 * The code for this is based on the SimpleRobot demo and
 * the code used from last year's competition.
 * The movement code should be extremely similar from last year.
 * Autonomous and OperatorControl methods are called either from the
 * driver station or the field controls.
 */
  
class MainRobot : public SimpleRobot {
	RobotDrive robotDrive;	// Robot drive system
	Joystick *stick1;		// Directional control
	Joystick *stick2;		// Lifting control
	bool fastSpeedEnabled;
	bool safetyModeOn;
	Timer timer;
	float debug;
	
	typedef enum
	{
		kPWMPort_1 = 1,
		kPWMPort_2 = 2,
		kPWMPort_3 = 3,
		kPWMPort_4 = 4,
		kPWMPort_5 = 5,
		kPWMPort_6 = 6,
		kPWMPort_7 = 7,
		kPWMPort_8 = 8,
		kPWMPort_9 = 9,
		kPWMPort_10 = 10,
		kPWMPort_11 = 11,
		kPWMPort_12 = 12,
		kPWMPort_13 = 13,
		kPWMPort_14 = 14
	} PWMPorts;
	
	typedef enum
	{
		kUSBPort_1 = 1,
		kUSBPort_2 = 2
	} USBPorts;
	
	typedef enum
	{
		kJSButton_1 = 1,
		kJSButton_2 = 2,
		kJSButton_3 = 3,
		kJSButton_4 = 4,
		kJSButton_5 = 5,
		kJSButton_6 = 6,
		kJSButton_7 = 7,
		kJSButton_8 = 8,
		kJSButton_9 = 9,
		kJSButton_10 = 10,
		kJSButton_11 = 11,
		kJSButton_12 = 12,
		kJSButton_13 = 13,
		kJSButton_14 = 14
	} JoyStickButtons;
	
	static const float ROTATION_CONSTANT = 1.0;
	static const float SPEED_DECREASE = 2.0;

	static const UINT32 LEFT_FRONT_MOTOR_PORT  = kPWMPort_1;
	static const UINT32 LEFT_REAR_MOTOR_PORT   = kPWMPort_2;
	static const UINT32 RIGHT_FRONT_MOTOR_PORT = kPWMPort_3;
	static const UINT32 RIGHT_REAR_MOTOR_PORT  = kPWMPort_4;

	static const UINT32 kMoveFastButton = kJSButton_1;

	static const UINT32 kEnableSafetyModeButton = kJSButton_11;
	static const UINT32 kDisableSafetyModeButton = kJSButton_10;
	
	// Button 3 (center button) turns clockwise,
	// Button 4 (left button) turns counterclockwise.
	static const UINT32 kRotateRightButton = kJSButton_3;
	static const UINT32 kRotateLeftButton = kJSButton_4;
	/**
	 * The above are the constants for the ports each motor goes into.
	 * See the method "MainRobot(void) in the below class for more.
	 */

	
public:
	/**
	 * Presumably the constructor function
	 * Todo:
	 * - Pending
	 */
	MainRobot(void):
		/**
		 * Explaination of mysterious numbers below:
		 * Each motor is connected to a jaguar, which is connected to a port.
		 * The constructor wants to know which ports control which motor. 
		 * Order it expects:
		 * Front Left Motor,
		 * Rear Left Motor,
		 * Front Right Motor,
		 * Rear Right Motor.
		 * See the constants at the top for the motor port numbers.
		 * The above is confirmed.
		 */
		robotDrive(LEFT_FRONT_MOTOR_PORT, LEFT_REAR_MOTOR_PORT, RIGHT_FRONT_MOTOR_PORT, RIGHT_REAR_MOTOR_PORT)
		{
			// This should be the constructor.
			//UpdateDashboard("Initializing...");
			GetWatchdog().SetExpiration(0.1);
			stick1 = new Joystick(kUSBPort_1); // Right joystick, direction
			stick2 = new Joystick(kUSBPort_2); // Left joystick, lifting
			
			fastSpeedEnabled = false;
			safetyModeOn = true;
			debug = 1.0;
		}
	
	/**
	 * Autonomous Mode
	 * Todo:
	 * Actually make it do something.
	 */
	void Autonomous(void) {
		//UpdateDashboard("Initializing autonomous...");
		GetWatchdog().SetEnabled(false);
		//UpdateDashboard("Starting autonomous.")
		while(IsAutonomous()) {
			// Placeholder for autonomous - just spins in a circle
			robotDrive.HolonomicDrive(0,90,0);
			Wait(0.5);
		}
		GetWatchdog().SetEnabled(true);
	}
	

	/**
	 * Operator Controlled Mode
	 * Todo:
	 * - Make more functions to add (such as the scissor lift)
	 */
	void OperatorControl(void) {
		//UpdateDashboard("Initializing operator control...");
		fastSpeedEnabled = false;
		safetyModeOn = true;
		timer.Start();
		//UpdateDashboard("Starting operator control.");
		while(IsOperatorControl()) {
			// Does nothing besides move around.
			GetWatchdog().Feed();
			FatalityChecks(stick1, stick2);
			OmniDrive(stick1);
			//ScissorLift(stick2);
			//MinibotDeploy;
			Wait(0.005);
		}
	}
	
	
	/**
	 * Scissor Lift Controllor
	 * Input = Data from Joystick 2
	 * Output = Scissor lift movement
	 * Todo:
	 * - Make the function/method (thing)
	 */
	// Scissor life code here
	//void ScissorLift(GenericHID *liftStick) {
		// Need to add.
		
	//}
	
	/**
	 * Fatality Check
	 * Input = Both joysticks
	 * Handles 
	 * - Joystick disconnects
	 * - Toggling safety mode
	 */
	void FatalityChecks(GenericHID * moveStick,
					  GenericHID * liftStick) {
		if (moveStick == NULL || liftStick == NULL) {
			// If joysticks are disconnected, terminate.
			//UpdateDashboard("ERROR: A joystick is disconnected.");
			wpi_fatal(NullParameter);
			return;
		}
		if (moveStick->GetRawButton(kEnableSafetyModeButton) ||
			liftStick->GetRawButton(kEnableSafetyModeButton)) {
			// Button 11 on both sticks enables safety mode
			//if (!safetyModeOn) {
				//UpdateDashboard("Safety mode on.");
			//}
			safetyModeOn = true;
		}
		if (moveStick->GetRawButton(kDisableSafetyModeButton) ||
			liftStick->GetRawButton(kDisableSafetyModeButton)) {
			// Button 10 on both sticks disables safety mode
			//if (safetyModeOn) {
				//UpdateDashboard("Safety mode off.");
			//}
			safetyModeOn = false;
		}
	}
	
	
	/**
	 * Minibot Deployer
	 * Input = Button push
	 * Output = Minibot deploys
	 * Todo:
	 * - Make it
	 */
	// Minibot deployment code here
	
	
	/**
	 * Input = Joystick data
	 * Output = Robot movement (controls mechanum wheels)
	 * Copied and pasted movement code from last year
	 * Altered so it uses the new buttons.
	 * Todo:
	 * - Try implementing the built-in momentum and dirction for
	 * holonomic drive instead of using the large math stuff.
	 */
	void OmniDrive(GenericHID *moveStick) {
		/**
		 * Fast speed only works when safety mode is disabled.
		 * Prevents robots from speeding dangerously during demos.
		 */
		if (safetyModeOn){
			fastSpeedEnabled = false;
		} else {
			if (moveStick->GetRawButton(kMoveFastButton)) {
				// Squeeze trigger to move fast
				//if (!fastSpeedEnabled) {
					//UpdateDashboard("Maximum Speed!");
				//}
				fastSpeedEnabled = true;
			} else {
				//if (fastSpeedEnabled) {
					//UpdateDashboard("Normal Speed.")
				//}
				// Release trigger to move slower
				fastSpeedEnabled = false;
			}
		}
		
		/**
		 * Quick lesson:
		 * (condition) ? (expression 1) : (expression 2)
		 * if condition is true, expression 1,
		 * else expression 2.
		 * Like a compact 'If' statement.
		 */
		float leftYValue = fastSpeedEnabled ? -moveStick->GetY() 
			  : -moveStick->GetY() / SPEED_DECREASE;
		float leftXValue = fastSpeedEnabled ? moveStick->GetX()
			  : moveStick->GetX() / SPEED_DECREASE;
		float magnitude = sqrt((leftYValue * leftYValue) 
						+ (leftXValue * leftXValue));
		//Above: Pythagorean Theorum to calculate distance.
		
		/**
		 * From here on down, presumably the code prevents the robot from
		 * drifting if somebody nudges the joystick.
		 */
		if (magnitude < 0.1)
			magnitude = 0;
		if (leftXValue > -0.1 && leftXValue < 0.1)
			leftXValue = 0.00001;
		if (leftYValue > -0.1 && leftYValue < 0.1)
			leftYValue = 0.00001;
		float direction = (180 / 3.14159) 
			              * atan(leftXValue/leftYValue);
		if (leftYValue < 0.0)
			direction += 180.0;
		
		// Starts rotation using buttons.  
		// Doesn't appear to use degrees.
		float rotationSpeed = (moveStick->GetThrottle() - 1.1) * -0.5 + 0.07;
		/**
		 * Above - uses the twiddly thing to adjust max rotation speed.
		 * Middle of the twiddly thing == 0, can lead to negative.
		 * Added 1.0 so twiddly thing is always positive.
		 */
		float rotation = (moveStick->GetRawButton(kRotateRightButton) ? 1.0 : 0.0) 
					   + (moveStick->GetRawButton(kRotateLeftButton) ? -1.0 : 0.0);
		if (rotation) {
			rotation = 
				rotationSpeed * rotation * (fastSpeedEnabled ? 
				ROTATION_CONSTANT : ROTATION_CONSTANT / SPEED_DECREASE);
		}
		if (rotation < 0.04 && rotation > -0.04) {
			// Just in case, prevents drifting.
			rotation = 0.0;
		}
		
		/**
		 * This is where the magic happens.
		 * Yeah.
		 */

		robotDrive.HolonomicDrive(magnitude, direction, rotation);
		/**
		 * Assuming that a input of '1.0' is normal.
		 * This is probably why a rotation of '6.0' created craziness.
		 */
	}
	
	/**
	 * Updates the dashboard
	 * Input = string to be displayed
	 * Todo:
	 * - Make new dashboard stuff
	 */
	
	/*
	void UpdateDashboard(char * output) {
		dds.printf(output);
		dds.finalize;
	}
	*/
};

START_ROBOT_CLASS(MainRobot);



//END OF DOCUMENT

/*-------------------- Recommended Maximum Length of Lines -------------------*/
