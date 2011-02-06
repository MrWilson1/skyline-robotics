/* **********************************************
 * Skyline High School Robotics Team
 * Spartabots, Team 2976
 * Main code for team robot
 * FRC Robotics competition 2011 - Logomotion
 ***********************************************/

/*-------------------- Recommended Maximum Length of Lines -------------------*/

#include "WPILib.h"
#include "math.h"
#include "string.h"



/**
 * The code for this is based on the SimpleRobot demo and
 * the code used from last year's competition.
 * The movement code should be extremely similar from last year.
 * Autonomous and OperatorControl methods are called either from the
 * driver station or the field controls.
 */
  
class MainRobot : public SimpleRobot {
	RobotDrive robotDrive;	// Robot drive system (wheels and whatnot)
	Joystick *stick1;		// Directional control
	Joystick *stick2;		// Lifting control
	bool fastSpeedEnabled;	
	bool safetyModeOn;		// Safety switch (mostly during demos)
	Timer timer;
	
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
	
	static const float SPEED_DECREASE = 0.5;

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
	
public:
	/**************************************
	 * MainRobot: (The constructor)
	 * TODO:
	 * - Pending
	 */
	MainRobot(void):
		/**
		 * Explaination of numbers below:
		 * Each motor is connected to a jaguar, which is connected to a port.
		 * The constructor wants to know which ports control which motor. 
		 * The order given is the correct order it expects.
		 * See the constants at the top for the motor port numbers.
		 */
		robotDrive(LEFT_FRONT_MOTOR_PORT, LEFT_REAR_MOTOR_PORT, 
		RIGHT_FRONT_MOTOR_PORT, RIGHT_REAR_MOTOR_PORT)
		{
			GetWatchdog().SetExpiration(0.1);
			stick1 = new Joystick(kUSBPort_1); // Right joystick, direction
			stick2 = new Joystick(kUSBPort_2); // Left joystick, lifting
			fastSpeedEnabled = false;
			safetyModeOn = true;
		}
	
	
	
	
	/**********************************************
	 * Autonomous:
	 * Input = Data from driver station or field.
	 * Output = Robot movement (hanging ubertubes)
	 * TODO:
	 * Actually make it do something.
	 */
	void Autonomous(void)
	{
		GetWatchdog().SetEnabled(false);
		while(IsAutonomous()) {
			// Placeholder for autonomous - just spins in a circle
			robotDrive.HolonomicDrive(0,90,0);
			Wait(0.5);
		}
		GetWatchdog().SetEnabled(true);
	}
	
	
	
	
	/**************************************
	 * OperatorControl:
	 * Input = Data from driver station or field
	 * Output = Robot movements 
	 * TODO:
	 * - Make more functions to add (such as the scissor lift)
	 */
	void OperatorControl(void)
	{
		fastSpeedEnabled = false;
		safetyModeOn = true;
		timer.Start();
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
	
	
	
	
	/*******************************************************
	 * ScissorLift:
	 * Input = Data from Joystick 2
	 * Output = Scissor lift movement
	 * TODO:
	 * - Make the function/method (thing)
	 */
	void ScissorLift(GenericHID *liftStick)
	{
		// Nothing here.
	}
	
	
	
	
	/*******************************************************
	 * FatalityChecks:
	 * Input = Both joysticks
	 * Output = None
	 * Handles 
	 * - Joystick disconnects
	 * - Toggling safety mode
	 * TODO:
	 * - Add checks for scissor lift and speed.
	 */
	void FatalityChecks(GenericHID * moveStick, GenericHID * liftStick)
	{
		if (moveStick == NULL || liftStick == NULL) {
			// If joysticks are disconnected, terminate.
			wpi_fatal(NullParameter);
			return;
		}
		if (moveStick->GetRawButton(kEnableSafetyModeButton) ||
			liftStick->GetRawButton(kEnableSafetyModeButton)) {
			// Button 11 on both sticks enables safety mode
			safetyModeOn = true;
		}
		if (moveStick->GetRawButton(kDisableSafetyModeButton) ||
			liftStick->GetRawButton(kDisableSafetyModeButton)) {
			// Button 10 on both sticks disables safety mode
			safetyModeOn = false;
		}
	}
	
	
	
	
	/************************************8
	 * Minibot Deployer
	 * Input = Button push
	 * Output = Minibot deploys
	 * TODO:
	 * - Write this.
	 */
	void MinibotDeploy(void)
	{
		// Nothing yet
	}
	
	
	
	
	/********************************************************
	 * OmniDrive:
	 * Input = Joystick data
	 * Output = Robot movement (controls mechanum wheels)
	 * Radically altered code from last year.
	 * Altered so it uses the new buttons.
	 * TODO:
	 * - Test to see if it works.
	 */
	void OmniDrive(GenericHID *moveStick)
	{
		/**
		 * Fast speed only works when safety mode is disabled.
		 * Prevents robots from speeding dangerously during demos.
		 */
		fastSpeedEnabled = false;
		if ((safetyModeOn == false) && moveStick->GetRawButton(kMoveFastButton)) {
			fastSpeedEnabled = true;
		}

		/**
		 * Finding magnitude, direction, and rotation (for holonomic drive)
		 * Magnitude == [-1.0 to 1.0]
		 * Direction == In degrees
		 * Rotation  == [-1.0 to 1.0] 
		 */
		float magnitude = fabs(stick1->GetMagnitude());
		float direction = stick1->GetDirectionDegrees();
		// I don't know exactly why, but using moveStick doesn't work above.
		// I think it's because of a combination of pointer weirdness and
		// how "'class GenericHID' has no member named 'X'".  Or something.
		// Also, 'fabs' returns the absolute value of a float.
		
		
		// Sorry about the magic numbers below.
		float rotationSpeed = (moveStick->GetThrottle() - 1.1) * -0.5 + 0.07;
		float rotationPress = int(moveStick->GetRawButton(kRotateRightButton)) 
							  - int(moveStick->GetRawButton(kRotateLeftButton));
		float rotation = rotationSpeed * rotationPress;
		
		/**
		 * Multiply everything by a fractional number if the safety catch
		 * hasn't been disabled yet.
		 */
		if (fastSpeedEnabled == false) {
			magnitude *= SPEED_DECREASE;
			rotation  *= SPEED_DECREASE;
		}
		
		/**
		 * Prevents values from drifting.
		 * If any values floats too close to zero, it just makes them zero.
		 */
		magnitude = (magnitude < 0.1) ? 0.0 : magnitude;
		rotation  = (fabs(rotation) < 0.04) ? 0.0 : rotation;
				
		/**
		 * This is where the magic happens.
		 * Yeah.
		 */
		robotDrive.HolonomicDrive(magnitude, direction, rotation);
	}
	
	
	
	
	/********************************************
	 * ReturnDashboard:
	 * Updates the dashboard
	 * Input = string to be displayed
	 * Output = true if dashboard was successfully updated, false if it wasn't.
	 * TODO:
	 * - Make new dashboard stuff
	 */
	bool ReturnDashboard (string * outputText)
	{
		// Nothing here.
		return false;
	}
};

START_ROBOT_CLASS(MainRobot);


/*-------------------- Recommended Maximum Length of Lines -------------------*/
// END OF DOCUMENT
