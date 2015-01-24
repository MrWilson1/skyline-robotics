package org.spartabots.frc2015.subsystem;

import org.spartabots.frc2015.Ports;
import org.spartabots.frc2015.util.Util;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Talon;

public class Drive {
	// Constants
	public static final double LEFT_ENCODER_TO_DISTANCE_RATIO = 0.1524; // in meters (15.24 cm)
	public static final double RIGHT_ENCODER_TO_DISTANCE_RATIO = 0.1524; // in meters (15.24 cm)
	
    // Drive Motors
    public RobotDrive m_drive;
    public Drive drive;
    public Talon traverse;
    
    // Encoders
    Encoder leftEc = new Encoder(0, 1, false);
    Encoder rightEc = new Encoder(2, 3, false);
    Encoder traverseEc = new Encoder(4, 5, false);
    
    // Gyro
    Gyro gyro;
    public static final double kp = 0.03;
	
    // Misc Variables
    double prevMove = 0;
    double prevRotate = 0;
    boolean speedMode = false;
    
    public Drive() {
    	init();
    }
    
    protected void init() {
    	m_drive = new RobotDrive(
    			Ports.RoboRIO.Pwm1, // Left
    			Ports.RoboRIO.Pwm0  // Right
    			);
        m_drive.setExpiration(0.1);
        traverse = new Talon(Ports.RoboRIO.Pwm2);
        
        gyro = new Gyro(1); // Dummy port
        
    }
    
    /* GYRO
     * -------------------------------------------------------------------------------- */
    public void resetGyro() {
    	gyro.reset();
    }
    
    public double getGyroAngle() {
    	return gyro.getAngle();
    }
    
    public double getGyroAngleRad() {
    	return (getGyroAngle() * Math.PI) / 180;
    }

    /* ENCODERS
     * -------------------------------------------------------------------------------- */
    public double getLeftEncoderDistance() {
    	return leftEc.get() * LEFT_ENCODER_TO_DISTANCE_RATIO;
    }
    
    public double getRightEncoderDistance() {
    	return rightEc.get() * RIGHT_ENCODER_TO_DISTANCE_RATIO;
    }
    
    public void resetEncoders() {
    	leftEc.reset();
    	rightEc.reset();
    }
    

    /* DRIVE
     * -------------------------------------------------------------------------------- */
    
    public void drive(double move, double rotate){
        m_drive.arcadeDrive(curveDrive(move, prevMove, true, 2), curveDrive(rotate, prevRotate, false, 3));
            
        // Set previous values for next loop
        prevRotate = rotate;
        prevMove = move;
    }
    
    private double curve(double value, double prevValue, double accelCurve){
        return prevValue + (value - prevValue) / accelCurve;
    }
    
    private double uCurve(double value){
        if(value < 0)
            return -(value * value);
        else
            return value * value;
    }
    
    public void resetPrev(){
        prevMove = 0;
        prevRotate = 0;        
    }
    
    public void toggleSpeedMode() {
    	this.speedMode = !speedMode;
    }
    
    public void setSpeedMode(boolean speedMode){
    	this.speedMode = speedMode;
    }
    
    public double curveDrive(double value, double prevValue, boolean isZeroStopped, double accelCurve){
        //velocity curve
        value = uCurve(value);
        
        //accel curve
        if(value == 0 && isZeroStopped)
            value = 0;
        else
            value = curve(value, prevValue, accelCurve);
        value = Util.limit(value, -1, 1);
        
        if (speedMode)
            return value / 1.25;
        else
            return value / 2.0;
    }

	public void stop() {
		m_drive.arcadeDrive(0, 0);
	}
}