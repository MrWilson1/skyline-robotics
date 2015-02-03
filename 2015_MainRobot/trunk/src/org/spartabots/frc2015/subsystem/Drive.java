package org.spartabots.frc2015.subsystem;

import org.spartabots.frc2015.util.Constants;
import org.spartabots.frc2015.util.Util;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;

public class Drive extends Subsystem {	
    // Drive Motors
    public RobotDrive m_drive;
    public Drive drive;
    public Talon traverse;
    
    // Encoders
    public Encoder leftEc = new Encoder(Constants.LEFT_EC_A, Constants.LEFT_EC_B, false, EncodingType.k1X);
    public Encoder rightEc = new Encoder(Constants.RIGHT_EC_A, Constants.RIGHT_EC_B, false);
    public Encoder traverseEc = new Encoder(Constants.TRAVERSE_EC_A, Constants.TRAVERSE_EC_B, false);
    
    // Built-in accelerometer
    public Accelerometer accel = new BuiltInAccelerometer(Accelerometer.Range.k4G); 
    
    // Gyro
    public Gyro gyro;
	
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

    /* ACCELEROMETER
     * -------------------------------------------------------------------------------- */
    public double getAccelX() {
    	return accel.getX();
    }
    
    public double getAccelY() {
    	return accel.getY();
    }
    
    public double getAccelZ() {
    	return accel.getZ();
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
    	return leftEc.getRaw() * Constants.LEFT_EC_ENCODER_TO_FEET_RATIO;
    	// (leftEc.getRaw() / 360) * (Constants.WHEEL_CIRCUMFERENCE/(26/15)); // * Constants.FEET_TO_METERS;
    }
    
    public double  getRightEncoderDistance() {
    	return rightEc.getRaw() * Constants.RIGHT_EC_ENCODER_TO_FEET_RATIO;
    }
    
    public void resetEncoders() {
    	leftEc.reset();
    	rightEc.reset();
    }
    

    /* DRIVE
     * -------------------------------------------------------------------------------- */
    
    public void drive(double move, double rotate){
    	double newMove = curveDrive(move, prevMove, true, 2);
    	double newRotate = curveDrive(rotate, prevRotate, false, 3);
        m_drive.arcadeDrive(newMove, newRotate);
            
        // Set previous values for next loop
        prevRotate = newRotate;
        prevMove = newMove;
    }
    
    private double curve(double value, double prevValue, double accelCurve){
        return prevValue + (value - prevValue) / accelCurve;
    }
    
    private double uCurve(double value){
        if (value >= 0)
        	if (value < 0.5)
        		return 2 * value * value;
        	else
        		return Math.sqrt((value - 0.5) * 2) / 2 + 0.5;
        else
        	return -uCurve(-value);
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