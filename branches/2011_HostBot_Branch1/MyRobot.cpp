/*************************************************
 * Skyline High School Robotics Team
 * Spartabots, Team 2976
 * Main code for team robot
 * FRC Robotics competition 2011 - Logomotion
 * 
 * This code is based on the SimpleRobot demo and the code used from last 
 * year's competition.
 * The movement code should be conceptually similar to last year's.
 * Autonomous() and OperatorControl() methods are automatically started
 * by either the driver station or by the field controls. 
 ************************************************/

/*-------------------- Recommended Maximum Length of Lines -------------------*/

#include "WPILib.h"
#include "math.h"

class MainRobot : public SimpleRobot {
	RobotDrive robotDrive;			// Robot drive system (wheels and whatnot)
	Joystick *stick1;				// Directional control
	Joystick *stick2;				// Lifting control
	Timer timer;					// The only timer
	
	Victor *deployMotor;			// The motor that deploys the minibot
	DigitalInput *deployFarLimit;	// The outer limit for minibot deployment
	DigitalInput *deployNearLimit;	// The closer limit for minibot.
	
	Victor *liftMotor;				// Controls the lift
	DigitalInput *liftHighLimit;	// The high limit for the lift
	DigitalInput *liftLowLimit;		// The lower limit for the lift
	
	DigitalInput *leftCam;			// The cameras for autonomous.
	DigitalInput *middleCam;		// Left camera follows the line.
	DigitalInput *rightCam;			// Left and right from robot's perspective.
	
	bool isFastSpeedOn;			// Normal or fast speed?
	bool isSafetyModeOn;		// Safety switch.
	bool isLiftHigh;			// Safety switch, can automatically disable.
	float currentHeight;		// Measured in inches.
	float listOfHeights [5];	// The various heights for the scissor-lift.
	bool isDoingPreset;			// Is the lift moving automatically?
	int currentPreset;			// Where the lift is going (preset).
	bool isThisOld;
	
	typedef enum			// The ports on the digital sidecar (motors)
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
		kPWMPort_10 = 10
	} PWMPorts;
	
	typedef enum			// The ports for the digital IO (limit switches)
	{
		kDigitalIO_1 = 1,
		kDigitalIO_2 = 2,
		kDigitalIO_3 = 3,
		kDigitalIO_4 = 4,
		kDigitalIO_5 = 5,
		kDigitalIO_6 = 6,
		kDigitalIO_7 = 7,
		kDigitalIO_8 = 8,
		kDigitalIO_9 = 9,
		kDigitalIO_10 = 10,
		kDigitalIO_11 = 11,
		kDigitalIO_12 = 12,
		kDigitalIO_13 = 13,
		kDigitalIO_14 = 14
	} DigitalIO;
	
	typedef enum			// For the joysticks.
	{
		kUSBPort_1 = 1,
		kUSBPort_2 = 2
	} USBPorts;
	
	typedef enum			// Buttons on the joystick.
	{
		kJSButton_1 = 1,	// Trigger.
		kJSButton_2 = 2,	// On top of stick (bottom)
		kJSButton_3 = 3,	// On top of stick (middle)
		kJSButton_4 = 4,	// On top of stick (left)
		kJSButton_5 = 5,	// On top of stick (right)
		kJSButton_6 = 6,	// Bottom left (further away)
		kJSButton_7 = 7,	// Bottom right (closer)
		kJSButton_8 = 8,	// Bottom front left button
		kJSButton_9 = 9,	// Bottom front right button
		kJSButton_10 = 10,	// Bottom right (closer)
		kJSButton_11 = 11,	// Bottom riht (further away)
		kJSButton_12 = 12,	// Doesn't exist
		kJSButton_13 = 13,	// Doesn't exist
		kJSButton_14 = 14	// Doesn't exist
	} JoyStickButtons;
	
	// Port (PWM) assignments
	static const UINT32 LEFT_FRONT_MOTOR_PORT	= kPWMPort_1;
	static const UINT32 LEFT_REAR_MOTOR_PORT	= kPWMPort_2;
	static const UINT32 RIGHT_FRONT_MOTOR_PORT	= kPWMPort_3;
	static const UINT32 RIGHT_REAR_MOTOR_PORT	= kPWMPort_4;
	static const UINT32 LIFT_MOTOR_PORT			= kPWMPort_5;
	static const UINT32 MINIBOT_DEPLOY_PORT		= kPWMPort_6;
	static const UINT32 LEFT_CAMERA_PORT		= kPWMPort_7;
	static const UINT32 MIDDLE_CAMERA_PORT		= kPWMPort_8;
	static const UINT32 RIGHT_CAMERA_PORT		= kPWMPort_9;
	
	// Digital IO assignments
	static const UINT32 HIGH_LIFT_DIO			= kDigitalIO_1;
	static const UINT32 LOW_LIFT_DIO			= kDigitalIO_2;
	static const UINT32 FAR_DEPLOY_DIO			= kDigitalIO_3;
	static const UINT32 NEAR_DEPLOY_DIO			= kDigitalIO_4;
	
	// Joystick assignments
	static const UINT32 MOVE_JOYSTICK_USB		= kUSBPort_1;
	static const UINT32 LIFT_JOYSTICK_USB		= kUSBPort_2;
	
	// Button assignments (Scissor-lift)
	static const UINT32 PRESET_BOTTOM = kJSButton_2;	// Botton top button
	static const UINT32 PRESET_PEG_1 = kJSButton_4;		// Left top button
	static const UINT32 PRESET_PEG_2 = kJSButton_3; 	// Center top button
	static const UINT32 PRESET_PEG_3 = kJSButton_5; 	// Right top button
	
	// Button assignments (Both)
	static const UINT32 ENABLE_SAFETY_BUTTON = kJSButton_6;
	static const UINT32 DISABLE_SAFETY_BUTTON = kJSButton_7;
	
	// Button assignments (Driving)
	static const UINT32 MOVE_FAST_BUTTON = kJSButton_1;
	static const UINT32 ROTATE_RIGHT_BUTTON = kJSButton_3; // Clockwise
	static const UINT32 ROTATE_LEFT_BUTTON = kJSButton_4;  // Counter-clockwise
	static const UINT32 EXTEND_MINIBOT_BUTTON = kJSButton_11;
	static const UINT32 RETRACT_MINIBOT_BUTTON = kJSButton_10;
	// ROTATE_RIGHT_BUTTON:	The center button, rotates clockwise.
	// ROTATE_LEFT_BUTTON:	The left button, rotates counter-clockwise.
	// The counterclockwise button was mapped to the center button because
	// mapping it to the right button would force the thumb to move too much.
	
	// General constants
	static const float ROBOT_HEIGHT 	= 36.5;		// More accurate.
	static const float GAMEPLAY_TIME 	= 120.0;
	static const float SPEED_DECREASE 	= 0.5;
	static const float DELAY_VALUE 		= 0.01;		// In seconds
	// ROBOT_HEIGHT:	Measures from the floor to the height of the scissors
	// 					when fully compressed, in inches.  
	// GAMEPLAY_TIME: 	How long teleoperated mode lasts (in seconds)
	// SPEED_DECREASE:	The factor by which the speed should decrease in normal
	// 					mode.  Multiply the output, not divide.
	// DELAY VALUE:		How long the robot should wait in movement loops.
	//					Motors have a minimum update speed.
	
	// Autonomous constants
	static const float FAST_AUTO_TIME = 10.0;
	static const float AUTO_CORRECTION  = 0.1;
	static const int MAX_NO_LINE = 5;				// Needs calibration
	static const int TARGET_PEG_AUTO = 3;
	// FAST_AUTO_TIME:	The time in seconds the robot is allowed to drive at
	// 					maximum speed.  The robot must eventually slow down
	// 					to avoid running into a pole
	// AUTO_CORRECTION:	The value at which the robot will attempt correcting
	// 					itself when it diverges from the line.
	// MAX_NO_LINE:		How many iterations the robot can go without detecting
	// 					a line before shutting down (gone rogue)
	// TARGET_PEG_AUTO: The chosen target peg.  May have to be turned into a 
	// 					variable if switches are incorporated.
	
	// Lift contants
	static const float SAFETY_HEIGHT = 77.0;		// Probably inaccurate
	static const float TURN_TRANSFORM = 0.5;		// Debug value
	// SAFETY_HEIGHT:	When the lift exceeds this height (in inches), the 
	//					robot is deemed too top-heavy to move at high speeds.
	// TURN_TRANSFORM:	Transforms the wanted distance to the correct amount
	// 					of motor rotations.
	// 					To use:
	// 					Rotation = Distance / TURN_TRANSFORM;
	// 					Distance = Rotation * TURN_TRANSFORM;
	// 					Partially verified value.
	
	// Deployment constants
	static const float MINIBOT_DEPLOY_SPEED = 0.3;
		
public:
	/****************************************
	 * MainRobot: (The constructor)
	 * Mandatory method.
	 * TODO:
	 * - Tweak anything related to the scissor lift - verify values.
	 * - Find out how to configure Victor.
	 */
	MainRobot(void):
		// Below: The constructor needs to know which port connects to which
		// motor so it can control the Jaguars correctly.
		// See constants at top.
		robotDrive(LEFT_FRONT_MOTOR_PORT, LEFT_REAR_MOTOR_PORT, 
		RIGHT_FRONT_MOTOR_PORT, RIGHT_REAR_MOTOR_PORT)
		{
			SmartDashboard::init();
			Watchdog();
			Watchdog().SetExpiration(0.1);  				// In seconds.
			stick1 = new Joystick(MOVE_JOYSTICK_USB); 		// Joystick 1
			stick2 = new Joystick(LIFT_JOYSTICK_USB);		// Joystick 2
			
			deployMotor = new Victor(MINIBOT_DEPLOY_PORT);
			deployFarLimit =  new DigitalInput(FAR_DEPLOY_DIO);
			deployNearLimit = new DigitalInput(NEAR_DEPLOY_DIO);
			
			liftMotor = new Victor(LIFT_MOTOR_PORT);
			liftHighLimit = new DigitalInput(HIGH_LIFT_DIO);
			liftLowLimit =  new DigitalInput(LOW_LIFT_DIO);
			
			leftCam = 	new DigitalInput(LEFT_CAMERA_PORT);
			middleCam = new DigitalInput(MIDDLE_CAMERA_PORT);
			rightCam = 	new DigitalInput(RIGHT_CAMERA_PORT);
			
			// The wiring was inverted on the left motors, so the below
			// is necessary.
			robotDrive.SetInvertedMotor(RobotDrive::kFrontLeftMotor, true);
			robotDrive.SetInvertedMotor(RobotDrive::kRearLeftMotor, true);
			
			isFastSpeedOn = false;
			isSafetyModeOn = true;
			isLiftHigh = false;
			// isSafetyModeOn:  Controlled by the driver -- should only have to
			// 					choose once.
			// isLiftHigh: 		Controlled by the program -- turns true only 
			//					when height is too high, otherwise turns false.
			
			currentHeight = 0.0;
			
			// Height of the pegs offset by the robot height (in inches).
			listOfHeights[0] = 0;
			listOfHeights[1] = 39.0  - ROBOT_HEIGHT;
			listOfHeights[2] = 77.0  - ROBOT_HEIGHT;
			listOfHeights[3] = 115.0 - ROBOT_HEIGHT;
			listOfHeights[4] = 0.0;  	// Zero-terminated just in case.
			isDoingPreset = false;
			currentPreset = 0;
			
			Watchdog().SetEnabled(true);
			UpdateDashboard("TestingTestingTestingTesting"
							"TestingTestingTestingTestingTesting");
			UpdateDashboard("Finished initializing.");
		}
	
	
	
	
	/****************************************
	 * Autonomous:
	 * Input = Data from driver station or field.
	 * Output = Robot movement (hanging ubertubes)
	 * Mandatory method.
	 * Line tracks edge.  Line should be under left camera only.
	 * If line detected by middle camera, adjust.
	 * Dead reckoning used.
	 * TODO:
	 * - Calibrate numbers and code
	 * - Test
	 * - Run through the flow of logic (double-checking)
	 */
	void Autonomous(void)
	{
		// Part 0 - initialization.
		Watchdog().Feed();
		timer.Reset();
		timer.Start();
		
		// If no line is detected, increments this.  If too high, robot stops.
		int safetyCount = 0;
		float rotation = 0.0;
		bool isAtEnd = false;
		bool isLiftDone = false;
		bool isError = false;
		const char *errorMessage = " ";
		UpdateDashboard("0: Starting Autonomous.");
		
		
		// Part 1 - Following the line.
		while(IsAutonomous()) {
			float magnitude = 1.0;
			float direction = 0.0;
			int lineState = GetLine();
			switch (lineState) {
				case 0:				// Nothing - too far right/going rogue
					++safetyCount;
					rotation = AUTO_CORRECTION;
				case 1:				// Left only - fine.
					rotation = 0.0;
					safetyCount = 0;
					break;
				case 2:				// Middle only - too far left.
				case 3:				// Left and middle - verging left.
				case 4:				// Right only - way too far left.
				case 5:				// Left and right - fork?
					// Handles all left-turning cases
					rotation = AUTO_CORRECTION * -1.0;
					safetyCount = 0;
					break;
				case 6:		// Middle and right - verging right.
					// Handles all right-turning cases
					rotation = AUTO_CORRECTION;
					break;
				case 7:
					// All sensors on - Hit end?
					safetyCount = 0;
					isAtEnd = true;
					break;
				default:
					// Are there more motors?
					isError = true;
					errorMessage = "ERROR: a1 - Too many line detectors?";
			}
			
			if (isAtEnd)
				break;
			if (safetyCount == MAX_NO_LINE) {
				isError = true;
				errorMessage = "ERROR: a1 - Drifted too far.";
			}
			if ((timer.Get() > FAST_AUTO_TIME) || isLiftHigh)
				magnitude *= SPEED_DECREASE;
			// Might not be good to reuse SPEED_DECREASE
			
			// Magic here - see method DriveHost for more info.
			robotDrive.HolonomicDrive(magnitude, direction, rotation);
			if (!isLiftDone)
				isError = AutoLift(TARGET_PEG_AUTO, isLiftDone);
			
			// Error-catching and checks
			if (isError)
				break;
			if (IsAutoDone()) {
				// Autonomous won't automatically quit when
				// it's operator control.
				return;
			}
			Watchdog().Feed();
			FatalityChecks(stick1, stick2);
			UpdateDashboard("1: Following the line...");
			Wait(DELAY_VALUE);
		}
		
		
		// Part two - if at the end, try raising the lift.
		if (isAtEnd) {
			while (!isLiftDone) {
				if (!isLiftDone)
					isError = AutoLift(TARGET_PEG_AUTO, isLiftDone);
				
				// Error-catching and checks
				if (isError) {
					errorMessage = "ERROR: a2 - Lift hit higher limit switch?";
					break;
				}
				if (IsAutoDone())
					return;
				Watchdog().Feed();
				FatalityChecks(stick1, stick2);
				UpdateDashboard("2: End of the line...");
				Wait(DELAY_VALUE);
			}
			
			
			// Part 3 - attempt lowering the lift.
			isLiftDone = false;
			while (!isLiftDone) {
				if (!isLiftDone)
					isError = AutoLift(0, isLiftDone);
				
				// Error-catching and checks
				if (isError) {
					errorMessage = "ERROR: a3 - Lift hit lower limit switch?";
					break;
				}
				if (IsAutoDone())
					return;
				Watchdog().Feed();
				FatalityChecks(stick1, stick2);
				UpdateDashboard("3: Tucking lift away...");
				Wait(DELAY_VALUE);
			}
		}
		
		
		// Part 4 - If any time left, wait here.  If errors, default to here.
		while(IsAutoDone()) {
			Watchdog().Feed();
			FatalityChecks(stick1, stick2);
			UpdateDashboard((isError) ? errorMessage : "4: Autonomous finished.");
			Wait(DELAY_VALUE);
		}
	}
	
	
	
	
	/****************************************
	 * OperatorControl:
	 * Input = Data from driver station or field
	 * Output = Robot movements
	 * Mandatory method. 
	 * TODO:
	 * None
	 */
	void OperatorControl(void)
	{
		timer.Reset();
		timer.Start();
		Watchdog().Feed();
		isFastSpeedOn = false;
		isSafetyModeOn = false;
		bool isLiftGood;
		UpdateDashboard("Starting Operator Control");
		while(IsOperatorControl()) {
			FatalityChecks(stick1, stick2);
			DriveHost(stick1);
			isLiftGood = ManualLift(stick2);
			MinibotDeploy(stick1);
			
			Watchdog().Feed();
			UpdateDashboard(isLiftGood ? " " : "Scissor Error");
			Wait(DELAY_VALUE);
		}
	}
	
	
	
	
	/****************************************
	 * FatalityChecks:
	 * Input = Both joysticks, error codes from ManualLift
	 * Output = None
	 * Handles 
	 * - Joystick disconnects
	 * - Toggling safety mode
	 * TODO:
	 * - Find out how 'wpi_fatal' works
	 * - Replace NULL with '0'? (zero)
	 */
	void FatalityChecks(GenericHID *moveStick, GenericHID *liftStick)
	{
		// Terminate if a joystick is disconnected.
		if ((NULL == moveStick) || (NULL == liftStick)) {
			int badMove = (NULL == moveStick);
			int badLift = (NULL == liftStick);
			if (badMove && !badLift) {
				UpdateDashboard("ERROR: Stick 1 disconnected");
			} else if (!badMove && badLift) {
				UpdateDashboard("ERROR: Stick 2 disconnected");
			} else if (badMove && badLift) {
				UpdateDashboard("ERROR: Stick 1 and stick 2 disconnected");
			}
			wpi_fatal(NullParameter);
			return;
		}

		if (false == Watchdog().IsAlive()) {
			UpdateDashboard("ERROR: The watchdog died");
			Watchdog().Kill();
			wpi_fatal(NullParameter);
			return;
		}
		
		if (moveStick->GetRawButton(ENABLE_SAFETY_BUTTON) ||
			liftStick->GetRawButton(ENABLE_SAFETY_BUTTON)) {
			isSafetyModeOn = true;
		}
		if (moveStick->GetRawButton(DISABLE_SAFETY_BUTTON) ||
			liftStick->GetRawButton(DISABLE_SAFETY_BUTTON)) {
			isSafetyModeOn = false;
		}
		
		// If the scissor-lift is too high, it might topple at higher speeds.
		if (currentHeight > SAFETY_HEIGHT) {
			isLiftHigh = true;
		} else {
			isLiftHigh = false;
		}
	}
	
		

	
	/****************************************
	 * DriveHost:
	 * Input = Joystick data
	 * Output = Robot movement (controls mechanum wheels)
	 * Radically altered code from last year.
	 * Altered so it uses the new buttons.
	 * TODO:
	 * - Find out how to pass robotDrive here without breaking anything
	 *   (Not a necessary task).
	 */
	void DriveHost(GenericHID *moveStick)
	{		
		// Safety primarily to prevent toppling or for safer demos.
		isFastSpeedOn = false;
		if (moveStick->GetRawButton(MOVE_FAST_BUTTON)) {
			if ((false == isSafetyModeOn) && (false == isLiftHigh)) {
				isFastSpeedOn = true;
			}
		}	
		
		// Magnitude: [-1.0 to 1.0] - How far to travel.
		// Direction: In degrees	- Which way to travel.
		// Rotation : [-1.0 to 1.0] - How much to turn.
		// Joystick returns a float in range [-1.0 to 1.0] automatically.
		// Using moveStick will not compile.
		float magnitude = fabs(stick1->GetMagnitude());	// fabs = Float abs
		float direction = stick1->GetDirectionDegrees();
		direction = (direction < 0.0) ? direction + 360.0 : direction;
		float rotationSpeed = (moveStick->GetThrottle() - 1.1) * -0.5 + 0.07;
		float rotationPress = int(moveStick->GetRawButton(ROTATE_RIGHT_BUTTON)) 
							  - int(moveStick->GetRawButton(ROTATE_LEFT_BUTTON));
		float rotation = rotationSpeed * rotationPress;
			
		// Just in case - prevents values from being over 1.0 (absolute value)
		// Higher numbers cause motors to spin alarmingly fast.
		magnitude = (magnitude > 1.0) ?  1.0 : magnitude;
		rotation  = (rotation > 1.0)  ?  1.0 : rotation;
		rotation  = (rotation < -1.0) ? -1.0 : rotation;
		
		if (!isFastSpeedOn) {
			magnitude *= SPEED_DECREASE;
			rotation  *= SPEED_DECREASE;
		}
		
		// Prevents drift if values are too close to zero.
		magnitude = (magnitude < 0.1) ? 0.0 : magnitude;
		rotation  = (fabs(rotation) < 0.04) ? 0.0 : rotation;
		
		// This is where the magic happens.
		robotDrive.HolonomicDrive(magnitude, direction, rotation);
		
		SmartDashboard::Log(direction, "JS- Distance: ");
		SmartDashboard::Log(magnitude, "JS- Magnitude: ");
		SmartDashboard::Log(rotation, "JS- Rotation: ");
	}
	
	
	
	
	/****************************************
	 * ManualLift:
	 * Input = 	The motor for the lift (Victors only(?))
	 * 			Data from Joystick 2
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
	bool ManualLift(GenericHID *liftStick)
	{	
		// Chose the preset (button input)
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
		
		int absoluteInput;
		// Pushing joystick up returns a negative Y-value, oddly.
		float userInput = -liftStick->GetY();
		
		// User Input -- overrides presets if necessary.
		if (fabs(userInput) > 0.1) {
			absoluteInput = GetSign(userInput);
			userInput = (fabs(userInput) > 1.0) ? absoluteInput : userInput;
			
			// Making sure scissor-lift doesn't go too high or too low.
			if (liftHighLimit->Get() && (userInput > 0.0))
				return false;
			if (liftLowLimit->Get() && (userInput < 0.0))
				return false;
					
			// Override any presets.
			isDoingPreset = false;
			currentPreset = 0;
			
			// Positive = counter-clockwise spin (when facing the motor)
			// Counter-clockwise = rise.
			// Luckily, a positive value corresponds with a rise in height(?).
			liftMotor->Set(userInput);
			currentHeight += userInput * TURN_TRANSFORM;
		} else {
			userInput = 0.0;
		}
		
		// If presets were not overriden, continue moving.
		if (isDoingPreset) {
			int valueReturned = PresetLift(currentPreset);
			if (1 == valueReturned) {
				isDoingPreset = false;
			} else if (-1 == valueReturned){
				return false;	// Error returned.
			}
		}
		
		SmartDashboard::Log(userInput, "userInput: ");
		SmartDashboard::Log(absoluteInput, "absoluteInput: ");
		
		return true;			// No error.
	}
	
	
	
	
	/****************************************
	 * PresetLift:
	 * Input = 	Peg number
	 * Output = Scissor movement
	 * 			0  = Haven't hit the target yet.
	 * 			1  = I've hit the target!
	 * 			-1 = Something is seriously wrong. (Negative 1)
	 * TODO
	 * - Test
	 * - Calibrate numbers
	 */	
	int PresetLift(int pegChoice) {
		// Basic checks: make sure that movement is needed or that the lift
		// hasn't hit the limit switches (it should never, but just in case...)
		float targetHeight = listOfHeights[pegChoice];
		if (targetHeight == currentHeight)
			return 1;
		if (liftHighLimit->Get() || liftLowLimit->Get())
			return -1;
		
		// Checking to make sure the motor doesn't spin too much.
		float neededDirection = targetHeight - currentHeight;
		float motorTurn;
		if (fabs(neededDirection / TURN_TRANSFORM) > 1.0) {
			// If needed distance exceeds the maximum motor movement...
			motorTurn = (float)(GetSign(neededDirection));
			currentHeight += (motorTurn * TURN_TRANSFORM);
		} else {
			// If needed distance falls under the maximum motor movement...
			motorTurn = neededDirection / TURN_TRANSFORM;
			currentHeight += neededDirection;
		}
		
		liftMotor->Set(motorTurn);
		return (currentHeight == targetHeight) ? 1 : 0;
	}
	
	
	
	
	/****************************************
	 * AutoLift:
	 * Input 	= Target peg number
	 * 			  Pointer to bool value (if finished)
	 * 			  Bool error value (true returned if no error)
	 * Output 	= Scissor-lift movement
	 * 			  Changes bool value that was passed.
	 * 			  Returns true if no error, false if error.
	 * For autonomous only.
	 */
	bool AutoLift(int targetPeg, bool &isFinished) {
		int liftOutput = PresetLift(targetPeg);
		isFinished = (1 == liftOutput) ? true : false;
		return (-1 == liftOutput) ? false : true;
	}
	
	
	
	
	/****************************************
	 * Minibot Deployer
	 * Input =	Deployment motor
	 * 			Button push
	 * 			Further limit
	 * 			Closer limit
	 * Output = Minibot deploys
	 * TODO:
	 * - Write a better one
	 * - Find accurate motor numbers
	 * - Add delays to prevent motor from breaking?
	 */
	void MinibotDeploy(GenericHID *moveStick)
	{
		if (moveStick->GetRawButton(EXTEND_MINIBOT_BUTTON)) {
			if (!deployFarLimit->Get())
				deployMotor->Set(MINIBOT_DEPLOY_SPEED);
		} else if (moveStick->GetRawButton(RETRACT_MINIBOT_BUTTON)) {
			if (!deployNearLimit->Get())
				deployMotor->Set(-MINIBOT_DEPLOY_SPEED);
		}
	}
	
	
	
	
	/****************************************
	 * GetLine:
	 * Input 	= Left sensor,
	 * 			  Middle sensor,
	 * 			  Right sensor
	 * Output 	= Integer value of line (0-7).
	 * 				(leftCam? * 1) + (middleCam? * 2) + (rightCam? * 4)
	 * For autonomous only.
	 * TODO:
	 * None
	 */
	int GetLine(void)
	{
		int leftInput 	= leftCam->Get() 	? 1 : 0;
		int middleInput = middleCam->Get() 	? 2 : 0;
		int rightInput 	= rightCam->Get()	? 4 : 0;
		return leftInput + middleInput + rightInput;
	}
	
	
	
	
	/****************************************
	 * IsAutoDone:
	 * Input 	= None
	 * Output 	= True if no longer autonomous
	 * For autonomous only.
	 * This checks to see if autonomous is over.
	 * In case the the competition-broadcast-thing fails and doesn't
	 * send the single stating that Autonomous is over, this also
	 * provides some interrupts.
	 * Autonomous can be ended by turning safety off or by
	 * pushing a joystick nearly to the max (any direction)
	 * while holding the trigger down.
	 */
	bool IsAutoDone(void)
	{
		// Can disable autonomous by turning safety off.
		bool safetyKill = 	stick1->GetRawButton(DISABLE_SAFETY_BUTTON) ||
							stick2->GetRawButton(DISABLE_SAFETY_BUTTON);
		
		// Can disable autonomous by attempting to move at max speed.
		bool moveKill = 	(fabs(stick1->GetMagnitude()) > 0.9) &&
							(stick1->GetRawButton(MOVE_FAST_BUTTON));
		bool liftKill =		(fabs(stick2->GetMagnitude()) > 0.9) &&
							(stick2->GetRawButton(MOVE_FAST_BUTTON));
		
		// Disable if a signal is sent out by the driver station.
		bool systemKill = (false == IsAutonomous()) || IsOperatorControl();
		
		return safetyKill || moveKill || liftKill || systemKill;
	}

	
	
	
	/****************************************
	 * UpdateDashboard:
	 * Input = none
	 * Output = Safety mode
	 * 			Watchdog state
	 * 			Robot Speed
	 * 			System state (Autonomous or Teleoperated?)
	 * 			Robot state (Enabled or Disabled?)
	 * 			Timer
	 * 			Minibot alert
	 * Dependent on the 'SmartDashboard' class from the WPI library.
	 * TODO:
	 * - Test to see if this works.
	 */
	void UpdateDashboard(void)
	{	
		// Setup here (default values set):
		const char *watchdogCheck = "DEAD";
		const char *systemState = "???";
		const char *minibotStatus = "...";
		
		if (Watchdog().IsAlive())
			watchdogCheck = Watchdog().GetEnabled() ? "Enabled" : "DISABLED";
		
		if (IsOperatorControl()) {
			systemState = "Teleoperate";
		} else if (IsAutonomous()) {
			systemState = "Autonomous";
		}
		
		if (timer.Get() >= (GAMEPLAY_TIME - 15))
			minibotStatus = "Get Ready";
		if (timer.Get() >= (GAMEPLAY_TIME - 10))
				minibotStatus = "DEPLOY";
		
		// Safety info
		SmartDashboard::Log(isSafetyModeOn ? "ENABLED" : "Disabled", "Safety mode: ");
		SmartDashboard::Log(watchdogCheck, "Watchdog State: ");
		
		// Robot actions
		SmartDashboard::Log(isFastSpeedOn ? "Fast" : "Normal", "Speed: ");
		SmartDashboard::Log(currentHeight, "Current Lift Height: ");
		
		// Info about the field state
		SmartDashboard::Log(systemState, "System State: ");
		SmartDashboard::Log(IsEnabled() ? "Enabled" : "DISABLED", "Robot State: ");
		
		// Info about time
		SmartDashboard::Log(minibotStatus, "MINIBOT ALERT: ");
	}

	
	
	
	/****************************************
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

		SmartDashboard::Log(outputText, "Message:");
	}
	
	
	
	
	/****************************************
	 * GetSign:
	 * Input  = a float
	 * Output = if number is positive, returns 1
	 * 			if number is negative, returns -1
	 * 			if number equals zero, returns 0
	 */
	int GetSign(float numberInput)
	{
		return ((int)(numberInput > 0.0)) - ((int)(numberInput < 0.0));
	}
};

START_ROBOT_CLASS(MainRobot);


/*-------------------- Recommended Maximum Length of Lines -------------------*/
// END OF DOCUMENT
