﻿/***********************************************
 * Skyline High School Robotics Team
 * Spartabots, Team 2976
 * Main code for team robot
 * FRC Robotics competition 2011 - Logomotion
 ***********************************************/

/**
 * To increase the readability of the code, I recommend making sure that 
 * lines are not too long.  Please attempt to make each line of code
 * approximately 80 characters long at most, although it should be fine to 
 * go a few characters over if it makes more sense that way.
 * Underneath this comment should be another comment block that is exactly
 * 80 characters long.
 */

/*-------------------- Recommended Maximum Length of Lines -------------------*/

#include "WPILib.h"
#include "math.h"
//#include "DashboardDataFormat.h"

void OmniDrive(GenericHID*, GenericHID*);

/**
 * The code for this is based on the SimpleRobot demo and
 * the code used from last year's competition.
 * The movement code should be extremely similar from last year.
 * Autonomous and OperatorControl methods are called either from the
 * driver station or the field controls.
 */
  
class MainRobot : public SimpleRobot {
	RobotDrive myRobot;				// Robot drive system
	Joystick *stick1;		// Directional control
	Joystick *stick2;		// Lifting control
	//DashboardDataFormat *dds;
	
public:
	/**
	 * Presumably the constructor function
	 * Todo:
	 * - Find out what the 'myRobot' initialization does.
	 */
	MainRobot(void):
		/**
		 * Explaination of mysterious numbers below:
		 * Each number corresponds to either the jaguar
		 * that controls that motor (or something like that).
		 * Order:
		 * Front Left Motor,
		 * Rear Left Motor,
		 * Front Right Motor,
		 * Rear Right Motor.
		 * Numbers copied and pasted from last year, may need
		 * to be altered based on current configuration.
		 */
		myRobot(1, 3, 2, 4) {
			// This should be the constructor.
			//UpdateDashboard("Initializing...");
			GetWatchdog().SetExpiration(0.1);
			stick1 = new Joystick(1); // Right joystick, direction
			stick2 = new Joystick(2); // Left joystick, lifting
			//dds = new DashboardDataFormat();
		}
	
	bool fastSpeedEnabled = false;
	bool safetyModeOn = true;
	Timer timer;
	
	/**
	 * Autonomous Mode
	 * TODO:
	 * Actually make it do something.
	 */
	void Autonomous(void) {
		//UpdateDashboard("Initializing autonomous...");
		getWatchdog().SetEnabled(false);
		//UpdateDashboard("Starting autonomous.")
		while(IsAutonomous()) {
			// Placeholder for autonomous - just spins in a circle
			myRobot.HolonomicDrive(0,90,0);
			Wait(0.5)
		}
	}
	

	/**
	 * Operator Controlled Mode
	 * Todo:
	 * - Make more functions to add (such as the scissor lift)
	 */
	void OperatorControl(void) {
		//UpdateDashboard("Initializing operator control...");
		myRobot.SetInvertedMotor(RobotDrive::kFrontRightMotor, true);
		fastSpeedEnabled = false;
		timer.Start();
		GetWatchdog().SetEnabled(true);
		//UpdateDashboard("Starting operator control.");
		while(IsOperatorControl()) {
			// Does nothing besides move around.
			GetWatchdog().Feed();
			FatalityChecks(stick1, stick2);
			//ScissorLift(stick2);
			OmniDrive(stick1);
			Wait(0.005)
		}
	}
	
	
	/**
	 * Scissor Lift Controllor
	 * Input = Data from Joystick 2
	 * Output = Scissor lift movement
	 * TODO
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
		if (moveStick->GetRawKey(11) || liftStick->GetRawKey(11)) {
			// Button 11 on both sticks enables safety mode
			if (!safetyModeOn) {
				//UpdateDashboard("Safety mode on.");
			}
			safetyModeOn = true;
		}
		if (moveStick->GetRawKey(10) || liftStick->GetRawKey(10)) {
			// Button 10 on both sticks disables safety mode
			if (safetyModeOn) {
				//UpdateDashboard("Safety mode off.");
			}
			safetyModeOn = false;
		}
	}	
	
	
	/**
	 * Minibot Deployer
	 * Input = Button push
	 * Output = Minibot deploys
	 * TODO
	 * - Make it
	 */
	// Minibot deployment code here
	
	
	/**
	 * Input = Joystick data
	 * Output = Robot movement (controls mechanum wheels)
	 * Copied and pasted movement code from last year
	 * Altered so it (hopefully) uses the new buttons.
	 */
	void OmniDrive(GenericHID *moveStick) {
		if (stick1->GetRawButton(1)) {
			// Squeeze trigger to move fast
			if (!fastSpeedEnabled) {
				//UpdateDashboard("Maximum Speed!");
			}
			fastSpeedEnabled = true;
		} else {
			if (fastSpeedEnabled) {
				//UpdateDashboard("Normal Speed.")
			}
			// Release trigger to move slower
			fastSpeedEnabled = false;
		}
		
		/**
		 * Quick lesson:
		 * (condition) ? (expression 1) : (expression 2)
		 * if condition is true, expression 1,
		 * else expression 2.
		 * Like a compact 'If' statement.
		 */
		float leftYValue = fastSpeedEnabled ? -leftStick->GetY() 
			  : -leftStick->GetY() / 2;
		float leftXValue = fastSpeedEnabled ? leftStick->GetX()
			  : leftStick->GetX() / 2;
		float magnitude = sqrt((leftYValue * leftYValue) 
				            + (leftXValue * leftXValue));
		//Above: Pythagorean Theorum to calculate distance.
		
		/**
		 * From here on down, presumably the code prevents the robot from
		 * drifting if somebody nudges the joystick.
		 */
		if (magnitude < 0.1) {
			magnitude = 0;}
		if (leftXValue > -0.1 && leftXValue < 0.1) {
			leftXValue = 0.00001;}
		if (leftYValue > -0.1 && leftYValue < 0.1) {
			leftYValue = 0.00001;}
		float direction = (180 / 3.14159) 
			              * atan(leftXValue/leftYValue);
		if (leftYValue < 0.0){
			direction += 180.0}
		
		// Starts rotation using buttons.  6 degree turn.
		float rotation;
		if (moveStick->GetRawButton(4) || moveStick->GetRawButton(3)) {
			rotation = ((moveStick->GetRawButton(3) - 
						 moveStick->GetRawButton(4))*
						 6.0);	//6 degree turn either way.
		} else {
			rotation = 0.0;
		}
		if (rotation < 0.1 && rotation > -0.1) {
			// Just in case.
			rotation = 0.0;
		}
		myRobot.HolonomicDrive(magnitude, direction, rotation);
	}
	
	/**
	 * Updates the dashboard
	 * Input = string to be displayed
	 * Todo:
	 * - Verify that this works
	 * - Check why there is no 'DashboardDataSender'
	 *   file in the new documentation.
	 * - Make sure I used 'char' correctly in this case.
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