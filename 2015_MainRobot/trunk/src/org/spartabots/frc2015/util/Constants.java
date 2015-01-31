package org.spartabots.frc2015.util;

import org.spartabots.frc2015.Ports;

public class Constants {
	
	// DRIVE
	public static final int DRIVE_LEFT_PORT 	= Ports.RoboRIO.Pwm1;
	public static final int DRIVE_RIGHT_PORT 	= Ports.RoboRIO.Pwm0;
	public static final int TRAVERSE_PORT 		= Ports.RoboRIO.Pwm2;

	public static final double DRIVE_FAST_REDUCTION_FACTOR = 1.25;
	public static final double DRIVE_SLOW_REDUCTION_FACTOR = 1.85;
	
	// ELEVATOR
	public static final int ELEVATOR1_PORT 		= Ports.RoboRIO.Pwm3;
	public static final int ELEVATOR2_PORT 		= Ports.RoboRIO.Pwm4;
	
	public static final int REGULAR_EASE_CONSTANT = 2;
	
	// GYRO
	public static final int GYRO_PORT 			= Ports.RoboRIO.AnalogChannel1;
	public static final double GYRO_KP 			= 0.03;
	
	// COMPRESSOR AND SOLENOIDS
	public static final int COMPRESSOR_PORT 	= 0;
	
	// ENCODERS
	public static final int LEFT_EC_A 			= Ports.RoboRIO.Dio1;
	public static final int LEFT_EC_B 			= Ports.RoboRIO.Dio2;
	
	public static final int RIGHT_EC_A 			= Ports.RoboRIO.Dio0;
	public static final int RIGHT_EC_B 			= Ports.RoboRIO.Dio3;
	
	public static final int TRAVERSE_EC_A 		= Ports.RoboRIO.Dio4;
	public static final int TRAVERSE_EC_B 		= Ports.RoboRIO.Dio5;
	
	public static final double LEFT_EC_TO_DISTANCE_RATIO = 0.1524 * Math.PI; // meters
	public static final double RIGHT_EC_TO_DISTANCE_RATIO = 0.1524 * Math.PI; // meters
	
}