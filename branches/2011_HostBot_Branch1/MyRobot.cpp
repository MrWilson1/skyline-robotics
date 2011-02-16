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
	RobotDrive robotDrive;		// Robot drive system (wheels and whatnot)
	Joystick *stick1;			// Directional control
	Joystick *stick2;			// Lifting control
	Timer timer;				// The only timer.
	Victor *deployMotor;		// The motor that deploys the minibot
	
	Victor *scissorMotor;		// Controls the scissor-lift
	DigitalInput *leftCam;		// The cameras for autonomous.
	DigitalInput *middleCam;	// Left camera follows the line.
	DigitalInput *rightCam;		// Left and right from robot's perspective.
	
	bool isFastSpeedOn;			// Normal or fast speed?
	bool isSafetyModeOn;		// Safety switch.
	bool isScissorHigh;			// Safety switch, but can automatically disable.
	float currentHeight;		// Measured in inches.
	float listOfHeights [5];	// The various heights for the scissor-lieft.
	bool isDoingPreset;			// Is the scissor-lift moving automatically?
	int currentPreset;			// Where the scissor-lift is going (preset).
	
	typedef enum			// The ports on the digital sidecar
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
	
	
	//* START NORMAL (remove one parenthesis for debug)
	// Port assignments
	static const UINT32 LEFT_FRONT_MOTOR_PORT	= kPWMPort_1;
	static const UINT32 LEFT_REAR_MOTOR_PORT	= kPWMPort_2;
	static const UINT32 RIGHT_FRONT_MOTOR_PORT	= kPWMPort_3;
	static const UINT32 RIGHT_REAR_MOTOR_PORT	= kPWMPort_4;
	static const UINT32 SCISSOR_MOTOR_PORT		= kPWMPort_5;
	static const UINT32 MINIBOT_DEPLOY_PORT		= kPWMPort_6;
	static const UINT32 LEFT_CAMERA_PORT		= kPWMPort_7;
	static const UINT32 MIDDLE_CAMERA_PORT		= kPWMPort_8;
	static const UINT32 RIGHT_CAMERA_PORT		= kPWMPort_9;
	
	
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
	static const float ROBOT_HEIGHT = 36.5;		// More accurate.
	static const float GAMEPLAY_TIME = 120.0;
	static const float SPEED_DECREASE = 0.5;
	// ROBOT_HEIGHT:	Measures from the floor to the height of the scissors
	// 					when fully compressed, in inches.  
	// GAMEPLAY_TIME: 	How long teleoperated mode lasts (in seconds)
	// SPEED_DECREASE:	The factor by which the speed should decrease in normal
	// 					mode.  Multiply the output, not divide.
	
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
	
	// Scissor-lift contants
	static const float SAFETY_HEIGHT = 77.0;		// Probably inaccurate
	static const float TURN_TRANSFORM = 9.403125;	// Possibly inaccurate
	// SAFETY_HEIGHT:	When the scissor-lift exceeds this height (in inches),
	// 					the robot is deemed too top-heavy to move at high
	// 					speeds.
	// TURN_TRANSFORM:	Transforms the wanted distance to the correct amount
	// 					of motor rotations.
	// 					To use:
	// 					Rotation = Distance / TURN_TRANSFORM;
	// 					Distance = Rotation * TURN_TRANSFORM;
	// 					Partially verified value.
	
	// END NORMAL */
	
	
	/* DEBUG (add one parenthesis for debugging)
	// Port assignments
	UINT32 LEFT_FRONT_MOTOR_PORT;
	UINT32 LEFT_REAR_MOTOR_PORT;
	UINT32 RIGHT_FRONT_MOTOR_PORT;
	UINT32 RIGHT_REAR_MOTOR_PORT;
	UINT32 SCISSOR_MOTOR_PORT;
	UINT32 LEFT_CAMERA_PORT;
	UINT32 MIDDLE_CAMERA_PORT;
	UINT32 RIGHT_CAMERA_PORT;
	UINT32 MINIBOT_DEPLOY_PORT;
	
	// Button assignments (Scissor-lift)
	UINT32 PRESET_BOTTOM;		// Botton top button
	UINT32 PRESET_PEG_1;		// Left top button
	UINT32 PRESET_PEG_2; 		// Center top button
	UINT32 PRESET_PEG_3; 		// Right top button
	
	// Button assignments (Both)
	UINT32 MOVE_FAST_BUTTON;
	UINT32 ENABLE_SAFETY_BUTTON;
	UINT32 DISABLE_SAFETY_BUTTON;
	
	// Button assignments (Driving)
	UINT32 ROTATE_RIGHT_BUTTON; 		// Clockwise
	UINT32 ROTATE_LEFT_BUTTON;  		// Counter-clockwise
	UINT32 EXTEND_MINIBOT_BUTTON;
	UINT32 RETRACT_MINIBOT_BUTTON;
	
	// General constants
	float ROBOT_HEIGHT;
	float GAMEPLAY_TIME;
	float SPEED_DECREASE;
	
	// Autonomous constants
	float FAST_AUTO_TIME;
	float AUTO_CORRECTION;
	int MAX_NO_LINE;				// Needs calibration
	int TARGET_PEG_AUTO;
	
	// Scissor-lift contants
	float SAFETY_HEIGHT;			// Probably inaccurate
	float TURN_TRANSFORM;			// Possibly inaccurate
	// END DEBUG */
	
public:
	/****************************************
	 * MainRobot: (The constructor)
	 * Mandatory method.
	 * TODO:
	 * - Tweak anything related to the scissor lift - verify values.
	 * - Find out how motor inversion works.
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
			Watchdog();						// Initialization
			Watchdog().SetExpiration(0.1);  // Expiration in seconds.
			stick1 = new Joystick(kUSBPort_1); // Right joystick, direction
			stick2 = new Joystick(kUSBPort_2); // Left joystick, lifting
			deployMotor = new Victor(MINIBOT_DEPLOY_PORT);
			scissorMotor = new Victor(SCISSOR_MOTOR_PORT);
			leftCam = 	new DigitalInput(LEFT_CAMERA_PORT);
			middleCam = new DigitalInput(MIDDLE_CAMERA_PORT);
			rightCam = 	new DigitalInput(RIGHT_CAMERA_PORT);
			
			// So, the wiring was inverted (stupid mechanics).
			// Something has to give, and sadly, my code must be encumbered.
			// Stupid mechanics.
			// robotDrive.SetInvertedMotor(RobotDrive::kFrontRightMotor, true);
			
			isFastSpeedOn = false;
			isSafetyModeOn = true;
			isScissorHigh = false;
			// isSafetyModeOn is a boolean controlled by the driver.
			// They can decide to disable or enable it.
			// isScissorHigh is a boolean automatically controlled.
			// It only becomes true if the height is too high, and 
			// automatically disables if the height falls back unders.
			// The driver should just choose to disable or enable safety mode, 
			// then never have to worry about it ever again.
			
			currentHeight = 0.0;	// Dead reckoning.
			// The height of the pegs offset by the height of the robot.
			// In inches.
			listOfHeights[0] = 0;
			listOfHeights[1] = 39.0  - ROBOT_HEIGHT;
			listOfHeights[2] = 77.0  - ROBOT_HEIGHT;
			listOfHeights[3] = 115.0 - ROBOT_HEIGHT;
			listOfHeights[4] = 130.0 - ROBOT_HEIGHT;
			listOfHeights[5] = 0.0;  // Zero-terminated just in case.
			isDoingPreset = false;
			currentPreset = 0;
			
			/* DEBUG (add one parenthesis for debugging)
			// Port assignments
			LEFT_FRONT_MOTOR_PORT	= kPWMPort_1;
			LEFT_REAR_MOTOR_PORT	= kPWMPort_2;
			RIGHT_FRONT_MOTOR_PORT	= kPWMPort_3;
			RIGHT_REAR_MOTOR_PORT	= kPWMPort_4;
			SCISSOR_MOTOR_PORT		= kPWMPort_5;
			LEFT_CAMERA_PORT		= kPWMPort_6;
			MIDDLE_CAMERA_PORT		= kPWMPort_7;
			RIGHT_CAMERA_PORT		= kPWMPort_8;
			MINIBOT_DEPLOY_PORT		= kPWMPort_9;
			
			// Button assignments (Scissor-lift)
			PRESET_BOTTOM 	= kJSButton_2;		// Botton top button
			PRESET_PEG_1 	= kJSButton_4;		// Left top button
			PRESET_PEG_2 	= kJSButton_3; 		// Center top button
			PRESET_PEG_3 	= kJSButton_5; 		// Right top button
			
			// Button assignments (Both)
			MOVE_FAST_BUTTON 		= kJSButton_1;
			ENABLE_SAFETY_BUTTON 	= kJSButton_6;
			DISABLE_SAFETY_BUTTON 	= kJSButton_7;
			
			// Button assignments (Driving)
			ROTATE_RIGHT_BUTTON 	= kJSButton_3; 	// Clockwise
			ROTATE_LEFT_BUTTON 		= kJSButton_4;  // Counter-clockwise
			EXTEND_MINIBOT_BUTTON 	= kJSButton_11;
			RETRACT_MINIBOT_BUTTON 	= kJSButton_10;
			
			// General constants
			ROBOT_HEIGHT 	= 36.5;			// More accurate.
			GAMEPLAY_TIME 	= 120.0;
			SPEED_DECREASE 	= 0.5;
			
			// Autonomous constants
			FAST_AUTO_TIME 	= 10.0;
			AUTO_CORRECTION = 0.1;
			MAX_NO_LINE 	= 5;				// Needs calibration
			TARGET_PEG_AUTO = 3;
			
			// Scissor-lift contants
			SAFETY_HEIGHT 	= 77.0;			// Probably inaccurate
			TURN_TRANSFORM 	= 9.403125;		// Possibly inaccurate
			// END DEBUG */
			
			Watchdog().SetEnabled(true);
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
	 * - Tweak numbers
	 * - Test
	 * - Run through the flow of logic (double-checking)
	 */
	void Autonomous(void)
	{
		//Part 0 - initialization.
		Watchdog().Feed();
		timer.Reset();
		timer.Start();
		// If no line is detected, increments this.  If too high, robot stops.
		int safetyCount = 0;
		float rotation = 0.0;
		bool isAtEnd = false;
		bool isScissorDone = false;
		bool isError = false;
		UpdateDashboard("0: Starting Autonomous.");
		
		// Part 1 - Following the line.
		while(IsAutonomous()) { 
			int lineState = GetLine();
			// Default vars
			float magnitude = 1.0;
			float direction = 0.0;
			switch (lineState) {
				case 0:		// Nothing - too far right/going rogue
					++safetyCount;
					rotation = AUTO_CORRECTION;
				case 1:		// Left only - fine.
					rotation = 0.0;
					safetyCount = 0;
					break;
				case 2:		// Middle only - too far left.
				case 3:		// Left and middle - verging left.
				case 4:		// Right only - way too far left.
				case 5:		// Left and right - fork?
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
			}
			
			if (isAtEnd == true) {
				break;
			}
			
			if (safetyCount == MAX_NO_LINE) {
				isError = true;
			}
			
			if ((timer.Get() > FAST_AUTO_TIME) || isScissorHigh) {
				// May have to create second constant for this instead of
				// reusing the one below.
				magnitude *= SPEED_DECREASE;
			}
			
			// Magic here - see method OmniDrive for more info.
			robotDrive.HolonomicDrive(magnitude, direction, rotation);
			
			if (isScissorDone) {
				isError = ScissorAuto(TARGET_PEG_AUTO, isScissorDone);
			}
			
			// Error-catching
			if (isError) {
				SmartDashboard::Log("1: Failed while following line.", 
									"AUTONOMOUS ERROR: ");
				break;
			}
			
			// Checks
			if (IsAutoDone()) {
				// Autonomous won't automatically quit when
				// it's operator control.
				return;
			}
			Watchdog().Feed();
			FatalityChecks(stick1, stick2);
			UpdateDashboard("1: Following the line...");
			Wait(0.005);
		}
		
		
		// Part two - if at the end...
		if (isAtEnd) {
			while (false == isScissorDone) {
				// Raising scissor...
				
				if (false == isScissorDone) {
					isError == ScissorAuto(TARGET_PEG_AUTO, isScissorDone);
				}
				
				// Error-catching
				if (isError) {
					SmartDashboard::Log("2: Failed at end, while raising lift."
										,"AUTONOMOUS ERROR: ");
					break;
				}
				
				// Checks
				if (IsAutoDone()) {return;}
				Watchdog().Feed();
				FatalityChecks(stick1, stick2);
				UpdateDashboard("2: End of the line...");
				Wait(0.005);
			}
			
			// Part 3 - lowering the scissor-lift.
			isScissorDone = false;
			while (false == isScissorDone) {
				// Try lowering scissor
				if (false == isScissorDone) {
					isError = ScissorAuto(0, isScissorDone);
				}
				
				// Error-catching
				if (isError) {
					SmartDashboard::Log("3: Failed while lowering lift.", 
										"AUTONOMOUS ERROR: ");
					break;
				}
				
				
				// Checks
				if (IsAutoDone()) {return;}
				Watchdog().Feed();
				FatalityChecks(stick1, stick2);
				UpdateDashboard("3: Tucking scissor away...");
				Wait(0.005);
			}
		}
		
		// Part 4 - resting.
		while(IsAutoDone()) {
			// If there's still time left, wait here.
			// If any errors emerge, should default to here.
			
			Watchdog().Feed();
			FatalityChecks(stick1, stick2);
			UpdateDashboard((isError) ? "4: Waiting after error..." 
									  : "4: Autonomous finished.");
			Wait(0.005);
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
		UpdateDashboard("Starting Operator Control");
		while(IsOperatorControl()) {
			FatalityChecks(stick1, stick2);
			OmniDrive(stick1);
			bool isScissorGood = ScissorManual(stick2);
			MinibotDeploy(stick1);
			
			Watchdog().Feed();
			UpdateDashboard(isScissorGood ? " " : "Scissor Error");
			Wait(0.005);
		}
	}
	
	
	
	
	/****************************************
	 * FatalityChecks:
	 * Input = Both joysticks, error codes from ScissorManual
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
			wpi_fatal(NullParameter);
			return;
		}
		
		if (false == Watchdog().IsAlive()) {
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
			isScissorHigh = true;
		} else {
			isScissorHigh = false;
		}
	}
	
		

	
	/****************************************
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
		isFastSpeedOn = false;
		if (moveStick->GetRawButton(MOVE_FAST_BUTTON)) {
			if ((false == isSafetyModeOn) && (false == isScissorHigh)) {
				isFastSpeedOn = true;
			}
		}				
		
        /**
         * Quick lesson:
         * (condition) ? (expression 1) : (expression 2)
         * if condition is true, expression 1,
         * else expression 2.
         * Like a compact 'If' statement.
         */
        float leftYValue = isFastSpeedOn ? -moveStick->GetY() 
                  : -moveStick->GetY() / SPEED_DECREASE;
        float leftXValue = isFastSpeedOn ? moveStick->GetX()
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
        float rotation = (moveStick->GetRawButton(ROTATE_RIGHT_BUTTON) ? 1.0 : 0.0) 
                                   + (moveStick->GetRawButton(ROTATE_LEFT_BUTTON) ? -1.0 : 0.0);
        if (rotation) {
                rotation = 
                        rotationSpeed * rotation * (isFastSpeedOn ? 
                        1 : SPEED_DECREASE);
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
		
		
		/*
		// Safety primarily to prevent toppling or for safer demos.
		isFastSpeedOn = false;
		if (moveStick->GetRawButton(MOVE_FAST_BUTTON)) {
			if ((false == isSafetyModeOn) && (false == isScissorHigh)) {
				isFastSpeedOn = true;
			}
		}	
		
		// Magnitude: [-1.0 to 1.0] - How far to travel.
		// Direction: In degrees	- Which way to travel.
		// Rotation : [-1.0 to 1.0] - How much to turn.
		// Joystick returns a float in range [-1.0 to 1.0] automatically.
		// Using moveStick will not compile - not sure why.
		float magnitude = fabs(stick1->GetMagnitude());	// fabs = Float abs
		float direction = stick1->GetDirectionRadians();
		float rotationSpeed = (moveStick->GetThrottle() - 1.1) * -0.5 + 0.07;
		float rotationPress = int(moveStick->GetRawButton(ROTATE_RIGHT_BUTTON)) 
							  - int(moveStick->GetRawButton(ROTATE_LEFT_BUTTON));
		float rotation = rotationSpeed * rotationPress;
		
		// Just in case - prevents values from being over 1.0 (absolute value)
		// Higher numbers cause motors to spin alarmingly fast.
		magnitude = (magnitude > 1.0) ?  1.0 : magnitude;
		rotation  = (rotation > 1.0)  ?  1.0 : rotation;
		rotation  = (rotation < -1.0) ? -1.0 : rotation;
		
		if (false == isFastSpeedOn) {
			magnitude *= SPEED_DECREASE;
			rotation  *= SPEED_DECREASE;
		}
		
		// Prevents drift if values are too close to zero.
		magnitude = (magnitude < 0.1) ? 0.0 : magnitude;
		rotation  = (fabs(rotation) < 0.04) ? 0.0 : rotation;
		
		// This is where the magic happens.
		robotDrive.HolonomicDrive(magnitude, direction, rotation);
		
		//*/
		
		
		// For debugging purposes.
		SmartDashboard::Log(direction, "JS- Distance: ");
		SmartDashboard::Log(magnitude, "JS- Magnitude: ");
		SmartDashboard::Log(rotation, "JS- Rotation: ");
	}
	
	
	
	
	/****************************************
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
		
		// User Input -- overrides above if necessary.
		if (liftStick->GetY()) {
			float userInput = liftStick->GetY();
			
			// Making sure not to exceed range [1.0 to -1.0]
			int absoluteInput = GetSign(userInput);
			userInput = (fabs(userInput) > 1.0) ? absoluteInput : userInput;
			
			// Making sure scissor-lift doesn't go too high or too low.
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
		
		// If presets were not overriden, continue moving.
		if (isDoingPreset) {
			int valueReturned = ScissorPreset(currentPreset);
			if (1 == valueReturned) {
				isDoingPreset = false;
			} else if (-1 == valueReturned){
				return false;	// Error returned.
			}
		}
		
		return true;			// No error.
	}
	
	
	
	
	/****************************************
	 * ScissorPreset:
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
		
		// Making sure not to go too high or too low.
		// Shouldn't ever happen, but just in case...
		float predictedHeight = currentHeight + neededDirection;
		if (predictedHeight < listOfHeights[0] || predictedHeight > listOfHeights[4]) {
			return -1;
		}
		
		float motorTurn;
		if (fabs(neededDirection / TURN_TRANSFORM) > 1.0) {
			// If needed distance exceeds the maximum motor movement...
			motorTurn = GetSign(neededDirection);
			currentHeight += GetSign(neededDirection) * TURN_TRANSFORM;
		} else {
			// If needed distance falls under the maximum motor movement...
			motorTurn = neededDirection / TURN_TRANSFORM;
			currentHeight += neededDirection;
		}
		
		scissorMotor->Set(motorTurn);
		return (currentHeight == targetHeight) ? 1 : 0;
	}
	
	
	
	
	/****************************************
	 * ScissorAuto:
	 * Input 	= Target peg height
	 * 			  Pointer to bool value (if finished)
	 * 			  Bool error value (true returned if no error)
	 * Output 	= Scissor-lift movement
	 * 			  Changes bool value that was passed.
	 * For autonomous only.
	 */
	bool ScissorAuto(int targetPeg, bool &isFinished) {
		int scissorOutput = ScissorPreset(targetPeg);
		if (-1 == scissorOutput) {
			return false;
		} else {
			isFinished = (1 == scissorOutput) ? true : false;
		}
		return true;
	}
	
	
	
	
	/****************************************
	 * Minibot Deployer
	 * Input = Button push
	 * Output = Minibot deploys
	 * TODO:
	 * - Write a better one
	 * - Find accurate motor numbers
	 * - Add delays to prevent motor from breaking?
	 */
	void MinibotDeploy(GenericHID *moveStick)
	{
		if (moveStick->GetRawButton(EXTEND_MINIBOT_BUTTON)) {
			deployMotor->Set(1.0);
		} else if (moveStick->GetRawButton(RETRACT_MINIBOT_BUTTON)) {
			deployMotor->Set(-1.0);
		}
	}
	
	
	
	
	/****************************************
	 * GetLine:
	 * Input 	= None
	 * Output 	= Integer value of line (0-7).
	 * 				(leftCam? * 1) + (middleCam? * 2) + (rightCam? * 4)
	 * For autonomous only.
	 */
	int GetLine(void)
	{
		int leftInput 	= leftCam->Get() 	? 1 : 0;
		int middleInput = middleCam->Get() 	? 2 : 0;
		int rightInput 	= rightCam->Get()	? 4 : 0;
		int output = leftInput + middleInput + rightInput;
		return output;
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
		bool stickKill = moveKill || liftKill;
		
		// Disable if a signal is sent out by the driver station.
		bool systemKill = (false == IsAutonomous()) || IsOperatorControl();
		
		bool output = (safetyKill || stickKill) || systemKill;
		return output;
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
		// Setup here:
		const char *watchdogCheck, *systemState, *minibotStatus;
		if (Watchdog().IsAlive()) {
			watchdogCheck = Watchdog().GetEnabled() ? "Enabled" : "DISABLED";
		} else {
			watchdogCheck = "DEAD";
		}
		if (IsOperatorControl()) {
			systemState = "Teleoperate";
		} else if (IsAutonomous()) {
			systemState = "Autonomous";
		} else {
			systemState = "???";
		}
		
		// Safety info
		SmartDashboard::Log(isSafetyModeOn ? "WARNING: Enabled" : "Disabled", 
							"Safety mode: ");
		SmartDashboard::Log(watchdogCheck, "Watchdog State: ");
		
		// Robot actions
		SmartDashboard::Log(isFastSpeedOn ? "Fast" : "Normal", "Speed: ");
		SmartDashboard::Log(currentHeight, "Current Lift Height: ");
		
		// Info about the field state
		SmartDashboard::Log(systemState, "System State: ");
		SmartDashboard::Log(IsEnabled() ? "Enabled" : "DISABLED",
							"Robot State: ");
		
		// Info about time
		SmartDashboard::Log(GAMEPLAY_TIME - timer.Get(), "Time Left: ");
		if (timer.Get() >= (GAMEPLAY_TIME - 15)) {
			minibotStatus = (timer.Get() >= (GAMEPLAY_TIME - 10)) 
							 ? "DEPLOY" : "Get Ready";
			SmartDashboard::Log(minibotStatus, "MINIBOT ALERT: ");
		}
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
		return int(numberInput > 0.0) - int(numberInput > 0.0);
	}
};

START_ROBOT_CLASS(MainRobot);


/*-------------------- Recommended Maximum Length of Lines -------------------*/
// END OF DOCUMENT
