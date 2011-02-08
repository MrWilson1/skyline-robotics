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
 * The movement code should be conceptually similar to last year.
 * Autonomous and OperatorControl methods are called either from the
 * driver station or the field controls.
 */
  
class MainRobot : public SimpleRobot {
	RobotDrive robotDrive;	// Robot drive system (wheels and whatnot)
	Joystick *stick1;		// Directional control
	Joystick *stick2;		// Lifting control
	//Jaguar *scissorMotor;	// Motor for controlling the scissor lift
	Timer timer;
	
	bool fastSpeedEnabled;	
	bool safetyModeOn;		// Safety switch (mostly during demos)
	float currentHeight;
	float presetTurn;
	float listOfHeights [5];
	
	
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
	
	static const UINT32 LEFT_FRONT_MOTOR_PORT  = kPWMPort_1;
	static const UINT32 LEFT_REAR_MOTOR_PORT   = kPWMPort_2;
	static const UINT32 RIGHT_FRONT_MOTOR_PORT = kPWMPort_3;
	static const UINT32 RIGHT_REAR_MOTOR_PORT  = kPWMPort_4;
	static const UINT32 SCISSOR_MOTOR_PORT     = kPWMPort_5;
	
	static const UINT32 kMoveFastButton = kJSButton_1;

	static const UINT32 kEnableSafetyModeButton = kJSButton_11;
	static const UINT32 kDisableSafetyModeButton = kJSButton_10;
	
	static const UINT32 kRotateRightButton = kJSButton_3;
	static const UINT32 kRotateLeftButton = kJSButton_4;
	// Button 3 (center button) turns clockwise,
	// Button 4 (left button) turns counterclockwise.
	
	/* Scissor Lift
	 * FUDGE_FACTOR  = How close the lift can get to the peg.
	 * 				   Absolute value - similar to 'deadband'?
	 * MAXIMUM_TURN  = How much the motor can turn per loop.
	 * 				   Converts joystick output into the appropriate approximate
	 * 				   amount of motor turns.
	 * SAFETY_HEIGHT = The height where safety mode should turn on if the
	 * 				   scissor lift goes too high
	 */
	static const float FUDGE_FACTOR  = 0.2;
	static const float MAXIMUM_TURN  = 1.0;
	static const float SAFETY_HEIGHT = 5.0;
	
	static const float GAMEPLAY_TIME = 120.0;
	// How long teleoperated lasts (in seconds)
	static const float SPEED_DECREASE = 0.5;
	// The percent the speed should decrease when in normal speed mode.
	
public:
	/**************************************
	 * MainRobot: (The constructor)
	 * Mandatory method.
	 * TODO:
	 * - Configure anything related to scissor lift better.
	 * - Initialize the motor for the scissor lift.
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
			SmartDashboard::init();
			Watchdog();
			Watchdog().SetExpiration(0.1);  // Expiration in seconds.
			stick1 = new Joystick(kUSBPort_1); // Right joystick, direction
			stick2 = new Joystick(kUSBPort_2); // Left joystick, lifting
			//scissorMotor = new Jaguar(SCISSOR_MOTOR_PORT); // Should be obvious
			
			fastSpeedEnabled = false;
			safetyModeOn = true;
			currentHeight = 0.0;	// Later, use a function to check motor/encoder.
			listOfHeights[0] = 0.0;
			listOfHeights[1] = 2.5;
			listOfHeights[2] = 4.0;
			listOfHeights[3] = 7.5;
			listOfHeights[4] = 8.0;
			listOfHeights[5] = 0.0;  // Zero terminated just in case.
			UpdateDashboard("Finished initializing.");
		}
	
	
	
	
	/**********************************************
	 * Autonomous:
	 * Mandatory method.
	 * Input = Data from driver station or field.
	 * Output = Robot movement (hanging ubertubes)
	 * TODO:
	 * - Add and test line following code.
	 */
	void Autonomous(void)
	{
		Watchdog().SetEnabled(true);
		UpdateDashboard("Starting Autonomous.");
		while(IsAutonomous()) {
			// Placeholder for autonomous - just spins in a circle
			robotDrive.HolonomicDrive(0,0,0.3);
			Watchdog().Feed();
			Wait(0.005);
			UpdateDashboard();
			/*
			
			
			What I want to do:
			
			Declare function LINE (arguments) {
				Take input from cameras;
				Move foward while adjusting based on camera;
				Check distance left;
				if (At end of distance given) {
					Yell "END"
				} else if (Detected fork) {
					Yell "FORK"
				} else if (Detected no line) {
					Yell "ERROR";
				}
				return (distance left);
			}
			
			// (Start of Autonomous method here)
			// (Various initialization here)
			if (Line detected) {
				while (IsAutonomous()) {
					Call Function LINE;
					if (you hear anything) {
						break;
					}
				}
			} else { 
				yell "ERROR";
			}
			
			if (you hear "FORK") {
				Add rotation to right (?);
				while (IsAutonomous()) {
					Call Function LINE;
					if ("FORK" or "ERROR") {
						yell "ERROR";
						break;
					}
					if ("END" is heard) {
						Adjust rotation to face front
						break;
					}
				}
			}
			
			if (you hear "END"){
				Call function SCISSOR_PRESET(3);
			}
			
			if (you hear "ERROR") {
			Call Function UpdateDashboard ("Error");
			while (IsAutonomous()) {
				// Do nothing, wait
			}

			*/
		}
	}
	
	
	
	
	/**************************************
	 * OperatorControl:
	 * Mandatory method.
	 * Input = Data from driver station or field
	 * Output = Robot movements 
	 * TODO:
	 * - Make more functions to add (such as the scissor lift)
	 */
	void OperatorControl(void)
	{
		Watchdog().SetEnabled(true);
		fastSpeedEnabled = false;
		safetyModeOn = false;
		timer.Reset();
		timer.Start();
		UpdateDashboard("Starting Operator Control");
		while(IsOperatorControl()) {
			FatalityChecks(stick1, stick2);
			OmniDrive(stick1);
			//ScissorLift(stick2);
			//MinibotDeploy();
			Watchdog().Feed();
			Wait(0.005);
			UpdateDashboard();
		}
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
		if (Watchdog().IsAlive() == false) {
			// If something's wrong with the watchdog, KILL IT
			Watchdog().Kill();
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
		if (currentHeight > SAFETY_HEIGHT) {
			// If the scissor lift is too high, it drops the speed for safety.
			safetyModeOn = true;
		}
	}
	
		
	
	
	/********************************************************
	 * OmniDrive:
	 * Critical piece of code
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
		 * Direction == In degrees (?)
		 * Rotation  == [-1.0 to 1.0] 
		 */
		float magnitude = fabs(stick1->GetMagnitude());
		float direction = stick1->GetDirectionDegrees();
		// I don't know exactly why, but using moveStick won't compile.
		// I think it's because of a combination of pointer weirdness and
		// how "'class GenericHID' has no member named 'X'".  Or something.
		// Also, 'fabs' returns the absolute value of a float.
		
		
		// Sorry about the magic numbers below.
		float rotationSpeed = (moveStick->GetThrottle() - 1.1) * -0.5 + 0.07;
		float rotationPress = int(moveStick->GetRawButton(kRotateRightButton)) 
							  - int(moveStick->GetRawButton(kRotateLeftButton));
		float rotation = rotationSpeed * rotationPress;
		
		/**
		 * To prevent the motors from breaking, makes sure that magnitude
		 * and rotation don't exceed an absolute value of 1.0
		 * Any higher is scarily fast - feels like it'd break the motors.
		 */
		magnitude = (magnitude > 1.0) ?  1.0 : magnitude;
		rotation  = (rotation > 1.0)  ?  1.0 : rotation;
		rotation  = (rotation < -1.0) ? -1.0 : rotation;
		
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
		
		/**
		 * For debugging purposes.
		 */
		SmartDashboard::Log(direction, "JS- Distance: ");
		SmartDashboard::Log(magnitude, "JS- Magnitude: ");
		SmartDashboard::Log(rotation, "JS- Rotation: ");
	}
	
	
	
	
	/*******************************************************
	 * ScissorLift:
	 * Input = Data from Joystick 2
	 * Output = Scissor lift movement
	 * TODO:
	 * - Find out how to actually control and read a motor.
	 * - Calibrate MAXIMUM_TURN
	 * - Create something that finds height.
	 * - Move the code that controls the presets to another method
	 *   (so it can be called from autonomous)
	 */
	void ScissorLift(GenericHID *liftStick)
	{
		// Currently in pseudo-code.
		
		/*
		Defined earlier, in constructor...
		listOfHeights - an array 5 slots long.
			Zeroth slot  = lowest height
			First slot   = first peg
			Second slot  = second peg
			Third slot   = third peg
			Fourth slot  = maximum height allowed.
			Fifth slot   = contains zero, just in case
		presetTurn    - a persistant var, lasts throughout main loop.
		currentHeight - a persistant var, should be directly updated for accuracy.
						Returns a value consistant with MAXIMUM_TURN.
		MAXIMUM_TURN  - a constant, the maximum amount of turns the motor can do each loop
						Transforms a decimal from -1.0 to 1.0 into the correct amount of
						terms.  Value should be found by experimenting.
						Additionally, MAXIMUM_TERM should be able to convert the decimal
						into a measurable feet compatible with 'currentHeight'.
		FUDGE_FACTOR  - the number (in feet) of how close the lift can be to the peg
						and still be acceptable.
		SAFETY_HEIGHT - If the currentHeight goes over this float, then it is deemed
						too dangerous to go around at max speed.  Therefore, safety mode
						is turned on.
		MagicMotorTurn - Hypothical from the library, turns the motor.
		
		Actually, jaguar turns at rate of [-1.0 to 1.0].
		Therefore, one must transform the difference between the currentHeight
		and the desired height into the jaguar turning rate.
		Also, 'MAXIMUM_TURN' might have to be tweaked so the motor doesn't
		accidentally smash something up.

	{ // Start of class (commented for now)	
		if (liftStick->GetY()) {
			float userInput = liftStick->GetY() * MAXIMUM_TURN;
			float presetTurn = 0.0;		// Override any preset turning
			float tempHeight = userInput + currentHeight;
			if (tempHeight < listOfHeights[0])
				userInput = listOfHeights[0] - currentHeight;	// Don't go too low
			if (tempHeight > listOfHeights[4])
				userInput = listOfHeights[4] - currentHeight);	// Don't go too high
			MagicMotorTurn(userInput);
		} else {
			// Preset only when no joystick movement
			// Awkward structure underneath - can't think of a more elegant way yet.
			bool setNewPreset = false;
			if (liftStick->GetRawButton(2)) {
				preset = 0;
				setNewPreset = true;
			} else if (liftStick->GetRawButton(4)) {
				preset = 1;
				setNewPreset = true;
			} else if (liftStick->GetRawButton(3)) {
				preset = 2;
				setNewPreset = true;
			} else if (liftStick->GetRawButton(5)) {
				preset = 3;
				setNewPreset = true;
			} else {
				setNewPreset = false;
			}
			if (setNewPreset) {
				if (currentHeight < (listOfHeights[preset] - FUDGE_FACTOR)) {
					// Too low - adjusting.
					float heightNeeded = listOfHeights[preset] - currentHeight;
					if (heightNeeded > MAXIMUM_TURN) {
						presetTurn = MAXIMUM_TURN;
					} else {
						// Make sure I don't overshoot when going up.
						presetTurn = heightNeeded;
					}
				} else if (currentHeight > (listOfHeights[preset] + FUDGE_FACTOR)) {
					// Too high - adjusting.
					float heightNeeded = currentHeight - listOfHeights[preset];
					if (heightNeeded > MAXIMUM_TURN) {
						presetTurn = -MAXIMUM_TURN;
					} else {
						// Make sure I don't overshoot when going down.
						presetTurn = -heightNeeded;
					}
				} else {
					// I've fallen into the fudge factor zone.  Stop turning.
					presetTurn = 0.0;
				}
			} // Done finding the amount I need to turn (for preset)
			if (presetTurn) {
				MagicMotorTurn(presetTurn);
			}
		} // Done with finding inputs and moving motor for joystick and preset
		
		currentHeight = MagicHeightFinderAndConverter();
	}			
	*/
	}
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	/************************************
	 * Minibot Deployer
	 * Input = Button push
	 * Output = Minibot deploys
	 * TODO:
	 * - Find out how to use this.
	 */
	void MinibotDeploy(void)
	{
		// Nothing yet
	}
	
	
	
	
	/*********************************
	 * UpdateDashboard
	 * Base: Updates the dashboard
	 * Input = none
	 * Output = Safety mode
	 * 			Watchdog state
	 * 			Robot Speed
	 * 			System state (Autonomous or Teleoperated?)
	 * 			Robot state (Enabled or Disabled?)
	 * 			Timer
	 * 			Minibot alert
	 * Obviously dependent on the 'SmartDashboard' stuff from
	 * the WPI library.
	 * TODO:
	 * - Test to see if this works.
	 */
	void UpdateDashboard(void)
	{
		// Safety Info
		SmartDashboard::Log(safetyModeOn ? "WARNING: Enabled" : "Disabled", 
						"Safety mode: ");
		const char *watchdogCheck;
		if (Watchdog().IsAlive()) {
			watchdogCheck = Watchdog().GetEnabled() ? "Enabled" : "DISABLED";
		} else {
			watchdogCheck = "DEAD";
		}
		SmartDashboard::Log(watchdogCheck, "Watchdog State: ");
		
		// Info about what the robot is doing
		SmartDashboard::Log(fastSpeedEnabled ? "Fast" : "Normal",
							"Speed: ");
		
		// Info about the field state
		const char *systemState;
		if (IsOperatorControl()) {
			systemState = "Teleoperate";
		} else if (IsAutonomous()) {
			systemState = "Autonomous";
		} else {
			systemState = "???";
		}
		SmartDashboard::Log(systemState, "System State: ");
		SmartDashboard::Log(IsEnabled() ? "Enabled" : "DISABLED", "Robot State: ");
		
		SmartDashboard::Log(GAMEPLAY_TIME - timer.Get(), "Time Left: ");
		const char *minibotStatus;
		if (timer.Get() >= (GAMEPLAY_TIME - 15)) {
			minibotStatus = (timer.Get() >= (GAMEPLAY_TIME - 10)) 
							 ? "DEPLOY" : "Get Ready";
			SmartDashboard::Log(minibotStatus, "MINIBOT ALERT: ");
		}
	}
	
	
	
	
	/********************************************
	 * UpdateDashboard:
	 * Overloading: Updates the dashboard, but with text also.
	 * Input = string to be displayed.
	 * Output = See UpdateDashboard(void)
	 * 			String from program.
	 */
	void UpdateDashboard(const char *outputText)
	{
		// Call to base dashboard updater.
		UpdateDashboard();
		
		// User-given data
		SmartDashboard::Log(outputText, "Message:");
	}
};

START_ROBOT_CLASS(MainRobot);


/*-------------------- Recommended Maximum Length of Lines -------------------*/
// END OF DOCUMENT
