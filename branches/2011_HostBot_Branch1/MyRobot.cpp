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
	Victor *scissorMotor;
	DigitalInput *leftCam;	// The cameras for autonomous.
	DigitalInput *middleCam;
	DigitalInput *rightCam;
	
	bool fastSpeedEnabled;	
	bool safetyModeOn;		// Safety switch
	float currentHeight;
	float listOfHeights [5];
	bool isDoingPreset;
	int currentPreset;
	
	
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
	
	static const UINT32 LEFT_FRONT_MOTOR_PORT	= kPWMPort_1;
	static const UINT32 LEFT_REAR_MOTOR_PORT	= kPWMPort_2;
	static const UINT32 RIGHT_FRONT_MOTOR_PORT	= kPWMPort_3;
	static const UINT32 RIGHT_REAR_MOTOR_PORT	= kPWMPort_4;
	static const UINT32 SCISSOR_MOTOR_PORT		= kPWMPort_5;
	static const UINT32 LEFT_CAMERA_PORT		= kPWMPort_6;
	static const UINT32 MIDDLE_CAMERA_PORT		= kPWMPort_7;
	static const UINT32 RIGHT_CAMERA_PORT		= kPWMPort_8;
	
	static const UINT32 PRESET_BOTTOM = kJSButton_2;// Botton top button
	static const UINT32 PRESET_PEG_1 = kJSButton_4;	// Left top button
	static const UINT32 PRESET_PEG_2 = kJSButton_3; // Center top button
	static const UINT32 PRESET_PEG_3 = kJSButton_5; // Right top button
	
	static const UINT32 kMoveFastButton = kJSButton_1;
	static const UINT32 kEnableSafetyModeButton = kJSButton_11;
	static const UINT32 kDisableSafetyModeButton = kJSButton_10;
	static const UINT32 kRotateRightButton = kJSButton_3;
	static const UINT32 kRotateLeftButton = kJSButton_4;
	// Button 3 (center button) turns clockwise,
	// Button 4 (left button) turns counterclockwise.
	
	static const float FAST_AUTO_TIME = 300.0;
	// During autonomous, the time the robot drives at max speed.  In seconds.
	static const float LARGE_AUTO_CORRECT = 0.3;
	static const float SMALL_AUTO_CORRECT = 0.1;
	// The auto corrects are the rotational values for autonomous when it
	// verges away from the line.
	
	static const float SAFETY_HEIGHT = 77.0;
	static const float TURN_TRANSFORM = 9.403125;
	// Transforms wanted distance in speed to correct amount of motor rotations.
	// To use: Rotation = Distance / TURN_TRANSFORM
	//		   Distance = Rotation * TURN_TRANSFORM
	// Partially verified by experiment.
	static const float ROBOT_HEIGHT = 120.0;
	// In inches - currently guessed value.
	// Measures from floor up to the height of the scissors when they are
	// compressed.
	static const float GAMEPLAY_TIME = 24.0;
	// How long teleoperated lasts (in seconds)
	static const float SPEED_DECREASE = 0.5;
	// The percent the speed should decrease when in normal speed mode.
	
public:
	/**************************************
	 * MainRobot: (The constructor)
	 * Mandatory method.
	 * TODO:
	 * - Tweak anything related to the scissor lift - verify values.
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
			scissorMotor = new Victor(SCISSOR_MOTOR_PORT);
			leftCam = 	new DigitalInput(LEFT_CAMERA_PORT);
			middleCam = new DigitalInput(MIDDLE_CAMERA_PORT);
			rightCam = 	new DigitalInput(RIGHT_CAMERA_PORT);
			
			fastSpeedEnabled = false;
			safetyModeOn = true;
			currentHeight = 0.0;	// Later, use a function to check motor/encoder.
			listOfHeights[0] = 0;	// All in inches.
			listOfHeights[1] = 39.0  - ROBOT_HEIGHT;
			listOfHeights[2] = 77.0  - ROBOT_HEIGHT;
			listOfHeights[3] = 115.0 - ROBOT_HEIGHT;
			listOfHeights[4] = 130.0 - ROBOT_HEIGHT;
			listOfHeights[5] = 0.0;  // Zero-terminated just in case.
			isDoingPreset = false;
			currentPreset = 0;
			UpdateDashboard("Finished initializing.");
		}
	
	
	
	
	/**********************************************
	 * Autonomous:
	 * Mandatory method.
	 * Input = Data from driver station or field.
	 * Output = Robot movement (hanging ubertubes)
	 * Line tracks edge.  Line should be under left camera only.
	 * If line detected by middle camera, adjust.
	 * Dead reckoning used.
	 * TODO:
	 * - Add and test line-following code.
	 */
	void Autonomous(void)
	{
		Watchdog().SetEnabled(true);
		float magnitude;
		float direction;
		float rotation;
		UpdateDashboard("Starting Autonomous.");
		Watchdog().Feed();
		timer.Reset();
		timer.Start();
		while(IsAutonomous()) {
			/*
			int lineState = GetLine();
			// Default vars
			magnitude = 1.0;
			direction = 0.0;
			rotation = 0.0;
			switch lineState {
				case 1:
					// Left only - fine.
					break;
				case 2:
					// Middle only - too far left.
					rotation = LARGE_AUTO_CORRECTION * -1.0;
					break;
				case 3:
					// Left and middle - verging left.
					rotation = SMALL_AUTO_CORRECTION * -1.0;
					break;
				case 4:
					// Right only - way too far left.
					

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
	 * None
	 */
	void OperatorControl(void)
	{
		Watchdog().SetEnabled(true);
		fastSpeedEnabled = false;
		safetyModeOn = false;
		timer.Reset();
		timer.Start();
		bool scissorCheck;
		UpdateDashboard("Starting Operator Control");
		while(IsOperatorControl()) {
			FatalityChecks(stick1, stick2);
			OmniDrive(stick1);
			scissorCheck = ScissorManual(stick2);
			//MinibotDeploy();
			Watchdog().Feed();
			Wait(0.005);
			UpdateDashboard((scissorCheck == false) ? "Scissor Error" : " ");
		}
	}
	
	
	
	
	/*******************************************************
	 * FatalityChecks:
	 * Input = Both joysticks, error codes from ScissorManual
	 * Output = None
	 * Handles 
	 * - Joystick disconnects
	 * - Toggling safety mode
	 * TODO:
	 * None
	 */
	void FatalityChecks(GenericHID *moveStick, GenericHID *liftStick)
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
	 * 			false = Error of some kind (probably passed from ScissorPreset)
	 * 			true  = Everything is just dandy.
	 * TURN_TRANSFORM- Distance * TURN_TRANSFORMS == amount of motor turns
	 * Victor turns at rate of [-1.0 to 1.0]
	 * 
	 * TODO:
	 * - Test code
	 * - Test MAXIMUM_TURN, calibrate numbers
	 * - Use limit switches for max or min, not dead reckoning.
	 */
	bool ScissorManual(GenericHID *liftStick)
	{	
		// Preset choose		
		if (liftStick->GetRawButton(PRESET_BOTTOM)) {
			currentPreset = 0;
			isDoingPreset = true;
		} else if (liftStick->GetRawButton(PRESET_PEG_1)) {
			currentPreset = 1;
			isDoingPreset = true;
		} else if (liftStick->GetRawButton(PRESET_PEG_2)) {
			currentPreset = 2;
			isDoingPreset = true;
		} else if (liftStick->GetRawButton(PRESET_PEG_3)) {
			currentPreset = 3;
			isDoingPreset = true;
		}
		
		// User Input (it can override above.  Joystick comes first.)
		if (liftStick->GetY()) {
			// Get joystick values.
			float userInput = liftStick->GetY();
			
			// Make sure you don't go over 1.0 or under -1.0
			int absoluteInput = GetSign(userInput);
			userInput = (fabs(userInput) > 1.0) ? absoluteInput : userInput;
			
			// Make sure you don't go too high or too low.
			float predictedHeight = (userInput * TURN_TRANSFORM) + currentHeight;
			if (predictedHeight < listOfHeights[0]) {
				userInput = currentHeight * -1.0 / TURN_TRANSFORM;
			} else if (predictedHeight > listOfHeights[4]) {
				userInput = (listOfHeights[4] - currentHeight) / TURN_TRANSFORM;
			}
			
			// Override any presets.
			isDoingPreset = false;
			currentPreset = 0;
			
			// Magic
			scissorMotor->Set(userInput);
		}
		
		// Actually do presets
		if (isDoingPreset == true) {
			int valueReturned = ScissorPreset(currentPreset);
			if (valueReturned == 1) {
				isDoingPreset = false;
			} else if (valueReturned == -1){
				return false;	// Error returned.
			}
		}
		
		return true;			// Everything is dandy.
	}
	
	
	
	
	/***********************************
	 * ScissorPreset
	 * Input = Peg number
	 * Output = Scissor movement
	 * 			0  = Haven't hit the target yet.
	 * 			1  = I've hit the target!
	 * 			-1 = Something is seriously wrong. (Negative 1)
	 * TODO
	 * - Test
	 * - Calibrate numbers
	 */
	int ScissorPreset(int pegChoice) {
		float targetHeight = listOfHeights[pegChoice];
		if (targetHeight == currentHeight) {
			return 1;
		}
		float neededDirection = targetHeight - currentHeight;
		int rawDirection = GetSign(neededDirection);
		
		// Make sure I don't go too low or too high.
		// Shouldn't ever happen, but just in case...
		float predictedHeight = currentHeight + neededDirection;
		if (predictedHeight < listOfHeights[0] || predictedHeight > listOfHeights[4]) {
			return -1;
		}
		
		// Don't want to overshoot the target peg.
		float motorTurn;
		if (fabs(neededDirection / TURN_TRANSFORM) > 1.0) {
			// If turning the motor the max amount won't get me to the peg,
			// Just go the max amount.
			motorTurn = rawDirection;
			currentHeight = (rawDirection * TURN_TRANSFORM) + currentHeight;
		} else {
			// Else, just go the amount I need.
			motorTurn = neededDirection / TURN_TRANSFORM;
			currentHeight = neededDirection + currentHeight;
		}
		
		scissorMotor->Set(motorTurn);
		return (currentHeight == targetHeight) ? 1 : 0;
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
	
	
	
	
	/***********************************
	 * GetLine
	 * Input 	= None
	 * Output 	= Integer value of line.
	 * 				(leftCam? * 1) + (middleCam? * 2) + (rightCam? * 4)
	 * For autonomous only.
	 */
	int GetLine(void)
	{
		int leftInput = leftCam->Get() 		? 1 : 0;
		int middleInput = middleCam->Get() 	? 1 : 0;
		int rightInput = rightCam->Get()	? 1 : 0;
		int output = leftInput + (middleInput * 2) + (rightInput * 4);
		return output;
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
		SmartDashboard::Log(currentHeight, "Current Lift Height: ");
		
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
	 * TODO:
	 * - Test
	 */
	void UpdateDashboard(const char *outputText)
	{
		// Call to base dashboard updater.
		UpdateDashboard();
		
		// User-given data
		SmartDashboard::Log(outputText, "Message:");
	}
	
	
	
	
	/*********************************************
	 * GetSign
	 * Input  = a float
	 * Output = if number is positive, returns 1
	 * 			if number is negative, returns -1
	 * 			if number equals zero, returns 0
	 * Written just for convinience
	 */
	int GetSign(float numberInput)
	{
		return int(numberInput > 0.0) - int(numberInput > 0);
	}
	
	
	

	/*********************************************
	 * GetSign
	 * Input  = a integer
	 * Output = if number is positive, returns 1
	 * 			if number is negative, returns -1
	 * 			if number equals zero, returns 0
	 * Written just for convinience
	 * Overloads previous
	 */
	int GetSign(int numberInput)
	{
		return int(numberInput > 0.0) - int(numberInput > 0);
	}
};

START_ROBOT_CLASS(MainRobot);


/*-------------------- Recommended Maximum Length of Lines -------------------*/
// END OF DOCUMENT
