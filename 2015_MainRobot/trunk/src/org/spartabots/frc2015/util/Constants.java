package org.spartabots.frc2015.util;

import org.spartabots.frc2015.Ports;

public class Constants {
	
	// DRIVE MOTORS
	public static final int DRIVE_LEFT_PORT 	= Ports.RoboRIO.Pwm1;
	public static final int DRIVE_RIGHT_PORT 	= Ports.RoboRIO.Pwm0;
	public static final int TRAVERSE_PORT 		= Ports.RoboRIO.Pwm2;
	
	// ELEVATOR
	public static final int ELEVATOR1_PORT 		= Ports.RoboRIO.Pwm3;
	public static final int ELEVATOR2_PORT 		= Ports.RoboRIO.Pwm4;
	
	// GYRO
	public static final int GYRO_PORT 			= 1;
	public static final double GYRO_KP 			= 0.03;
	
	// COMPRESSOR AND SOLENOIDS
	public static final int COMPRESSOR_PORT 	= 0;
	
	// ENCODERS
	public static final int LEFT_EC_A 			= 0;
	public static final int LEFT_EC_B 			= 1;
	
	public static final int RIGHT_EC_A 			= 2;
	public static final int RIGHT_EC_B 			= 3;

	public static final int TRAVERSE_EC_A 		= 4;
	public static final int TRAVERSE_EC_B 		= 5;
}
