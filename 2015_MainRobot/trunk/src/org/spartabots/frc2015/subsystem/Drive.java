package org.spartabots.frc2015.subsystem;

import org.spartabots.frc2015.util.Constants;
import org.spartabots.frc2015.util.Util;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Talon;

public class Drive extends Subsystem {
	// Constants
	
    // Drive Motors
    public RobotDrive m_drive;
    public Drive drive;
    public Talon traverse;
    
    // Encoders
    Encoder leftEc = new Encoder(Constants.LEFT_EC_A, Constants.LEFT_EC_B, false);
    Encoder rightEc = new Encoder(Constants.RIGHT_EC_A, Constants.RIGHT_EC_B, false);
    Encoder traverseEc = new Encoder(Constants.TRAVERSE_EC_A, Constants.TRAVERSE_EC_B, false);
    
    // Gyro
    Gyro gyro;
	
    // Misc Variables
    double prevMove = 0;
    double prevRotate = 0;
    boolean speedMode = false;
    
    public Drive() {
    	super();
    }
    
    protected void init() {
    	m_drive = new RobotDrive(
    			Constants.DRIVE_LEFT_PORT,
    			Constants.DRIVE_RIGHT_PORT
    			);
        m_drive.setExpiration(0.1);
        traverse = new Talon(Constants.TRAVERSE_PORT);
        
        gyro = new Gyro(Constants.GYRO_PORT); // Dummy port
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
    	return leftEc.get() * Constants.LEFT_EC_TO_DISTANCE_RATIO;
    }
    
    public double getRightEncoderDistance() {
    	return rightEc.get() * Constants.RIGHT_EC_TO_DISTANCE_RATIO;
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
            return value / Constants.DRIVE_FAST_REDUCTION_FACTOR;
        else
            return value / Constants.DRIVE_SLOW_REDUCTION_FACTOR;
    }

	public void stop() {
		m_drive.arcadeDrive(0, 0);
	}
}