package org.spartabots.frc2015.util;

import org.spartabots.frc2015.Ports;

public class Constants {
	
	// DRIVE
	public static final int DRIVE_LEFT_PORT 	= Ports.RoboRIO.Pwm1;
	public static final int DRIVE_RIGHT_PORT 	= Ports.RoboRIO.Pwm0;
	public static final int TRAVERSE_PORT 		= Ports.RoboRIO.Pwm2;
	
	public static final double DRIVE_FAST_REDUCTION_FACTOR 			= 1.24;
	public static final double DRIVE_SLOW_REDUCTION_FACTOR 			= 1.45;
	
	public static final double ROTATE_FAST_REDUCTION_FACTOR 		= 1.15;
	public static final double ROTATE_SLOW_REDUCTION_FACTOR 		= 1.25;

	// WHEEL
	public static final double WHEEL_DIAMETER 			= 0.5;
	public static final double WHEEL_RADIUS				= WHEEL_DIAMETER/2;
	public static final double WHEEL_CIRCUMFERENCE 		= 0.5 * Math.PI;
	public static final double FEET_TO_METERS 			= 0.3048;
	
	// ELEVATOR
	public static final int ELEVATOR_PORT 								= Ports.RoboRIO.Pwm3;
	public static final int REGULAR_EASE_CONSTANT 						= 2;
	public static final double ELEVATOR_DOWN_FAST_REDUCTION_FACTOR 		= 1.55; // for slow mode
	public static final double ELEVATOR_DOWN_SLOW_REDUCTION_FACTOR 		= 1.85; // for slow mode
	public static final double ELEVATOR_UP_FAST_REDUCTION_FACTOR		= 1; // for slow mode
	public static final double ELEVATOR_UP_SLOW_REDUCTION_FACTOR		= 1; // for slow mode
	public static final double ELEVATOR_HEIGHT 							= 6; /* about */ 
	
	// GYRO
	public static final int GYRO_PORT 			= Ports.RoboRIO.AnalogChannel1;
	public static final double GYRO_KP 			= 0.175;
	
	// COMPRESSOR AND SOLENOIDS
	public static final int COMPRESSOR_PORT 	= 0;
	public static final int SOLENOID_EGRIP	 	= 0;
	
	// ENCODERS
	public static final int D_BELT_EC_A	 		= Ports.RoboRIO.Dio0; // belt
	public static final int D_BELT_EC_B 		= Ports.RoboRIO.Dio1;
	
	public static final int D_GEARBOX_EC_A 		= Ports.RoboRIO.Dio2; // gearbox
	public static final int D_GEARBOX_EC_B 		= Ports.RoboRIO.Dio3;
	
	public static final int ELEVATOR_EC_A		= Ports.RoboRIO.Dio5;
	public static final int ELEVATOR_EC_B		= Ports.RoboRIO.Dio6;
	
	public static final int TRAVERSE_EC_A 		= Ports.RoboRIO.Dio7;
	public static final int TRAVERSE_EC_B 		= Ports.RoboRIO.Dio8;
	
	// ENCODER RATIOS
	public static final double BELT_EC_ENCODER_TO_FEET_RATIO 		= 15.0/6878;
	public static final double GEARBOX_EC_ENCODER_TO_FEET_RATIO 	= 15.0/6878;
	public static final double TRAVERSE_EC_ENCODER_TO_FEET_RATIO 	= 5.0/9000;
	public static final double ELEVATOR_EC_ENCODER_TO_FEET_RATIO 	= 3.875/8716; 
	
	// LIMIT SWITCHES
	public static final int ELEVATOR_BOTTOM		= Ports.RoboRIO.Dio4; 
	public static final int ELEVATOR_TOP		= Ports.RoboRIO.Dio9;
}