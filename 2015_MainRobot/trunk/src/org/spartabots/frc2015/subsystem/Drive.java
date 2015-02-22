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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive extends Subsystem {	
    // Drive Motors
    public RobotDrive m_drive;
    public Talon traverse;
    public double driveEcOffset = 0;
    
    // Encoders
    public Encoder beltEc = new Encoder(Constants.D_BELT_EC_A, Constants.D_BELT_EC_B, false, EncodingType.k1X);
    public Encoder gearboxEc = new Encoder(Constants.D_GEARBOX_EC_A, Constants.D_GEARBOX_EC_B, false);
    public Encoder traverseEc = new Encoder(Constants.TRAVERSE_EC_A, Constants.TRAVERSE_EC_B, false);
    
    // Built-in accelerometer
    public Accelerometer accel = new BuiltInAccelerometer(Accelerometer.Range.k4G); 
    
    // Gyro
    public Gyro gyro;
    public double gyroOffset = 0;
    public boolean lockGyroOffsetChange = false;
    public boolean kpDisabled = false;
    public boolean kpOff = false;
    
	private final double waitForAdjust = 0.85; // in secodns
	private double driveStraightStartTime = 0;
	
	protected double gV = 0.0;
	private double gp = 0.0;
    private double gVpTime = 0;
    private double gVCheckInterval = 0.1;
	
    // Misc Variables
    double prevMove = 0;
    double prevRotate = 0;
    boolean speedMode = false;
	public boolean isDrivingStraight = false;
	public boolean isNotMoving = false;
    
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
        
        gyro = new Gyro(Constants.GYRO_PORT);
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
    
    public void setZeroHeading() {
    	if (!lockGyroOffsetChange)
    		this.gyroOffset = -gyro.getAngle();
    }
    
    public double getGyroAngle() {
    	return gyro.getAngle() + gyroOffset;
    }
    
    public double getGyroAngleRad() {
    	return (getGyroAngle() * Math.PI) / 180;
    }

    /* ENCODERS
     * -------------------------------------------------------------------------------- */
    
    public void setZeroEcDist() {
    	this.driveEcOffset = -(beltEc.getRaw() * Constants.BELT_EC_ENCODER_TO_FEET_RATIO);
    }
    
    public double getBeltEncoderDistance() {
    	return (beltEc.getRaw() * Constants.BELT_EC_ENCODER_TO_FEET_RATIO) + this.driveEcOffset;
    	// (leftEc.getRaw() / 360) * (Constants.WHEEL_CIRCUMFERENCE/(26/15)) * Constants.FEET_TO_METERS;
    }
    
    public double getGearboxEncoderDistance() {
    	return gearboxEc.getRaw() * Constants.GEARBOX_EC_ENCODER_TO_FEET_RATIO;
    }

    public double getTraverseEncoderDistance() {
    	return traverseEc.getRaw() * Constants.TRAVERSE_EC_ENCODER_TO_FEET_RATIO;
    }
    
    public void resetEncoders() {
    	beltEc.reset();
    	gearboxEc.reset();
    	traverseEc.reset();
    }
    
    
    /* DRIVE
     * -------------------------------------------------------------------------------- */
    public void pollGV() {
    	double t = robot.profile.getTime();
    	if (gVpTime == 0) {
    		gVpTime = t;
    	}
    	if (t > (gVpTime + gVCheckInterval)) {
    		gV = (this.getGyroAngle() - this.gp) / (t - gVpTime);
    	}
    }
    
    public void drive(double move, double rotate) {
    	double newMove = curveDrive(move, prevMove, true, 2, false);
    	double newRotate = curveDrive(rotate, prevRotate, false, 3, true);
		double angle = this.getGyroAngle();
		pollGV();
		
    	// LOGGING
    	// ----------------------------------------------------------------------
    	SmartDashboard.putNumber("New Movement", Math.round(newMove * 100.0) / 100.0);
    	SmartDashboard.putNumber("Rotation", rotate);
    	SmartDashboard.putNumber("New Rotation", newRotate);
    	
    	// CONDITIONS
    	// ----------------------------------------------------------------------
    	if (rotate == 0 && move != 0) { // not rotating, moving
    		SmartDashboard.putString("Drive state", "not rotating, moving");
    		if (!isDrivingStraight) {
    			this.setZeroHeading();
    			this.isDrivingStraight = true;
    			this.driveStraightStartTime = robot.profile.getTime();
    			this.kpOff = true;
    		} else if (kpOff) {
    			if ((robot.profile.getTime() - driveStraightStartTime) > waitForAdjust) {
        			this.setZeroHeading();
            		this.kpOff = false;	
    			}
    		}
    	} else {
    		this.isDrivingStraight = false;
    	}
    	
    	if (rotate != 0 && move == 0) { // rotating, not moving
    		SmartDashboard.putString("Drive state", "rotating, not moving");
    		this.kpOff = true;
    	}
    	if (rotate == 0 && move == 0) { // not moving at all
    		SmartDashboard.putString("Drive state", "not rotating, not moving");
    		if (!isNotMoving) {
    			this.setZeroHeading();
    			isNotMoving = true;
    		}
    		this.kpOff = true;
    	}
    	if (rotate != 0 && move != 0) { // rotating and moving
    		SmartDashboard.putString("Drive state", "rotating and moving");
    		this.kpOff = true;
    	}

    	// LOGGING
    	// ----------------------------------------------------------------------
    	SmartDashboard.putBoolean("kpOff", kpOff);
    	SmartDashboard.putBoolean("kpDisabled", kpDisabled);
    	
    	// DRIVE
    	// ----------------------------------------------------------------------
    	if (isDrivingStraight) {
    		SmartDashboard.putNumber("gV angle", Math.round(angle * 100.0) / 100.0);
    		double newRotate2 = newRotate + (-angle * SmartDashboard.getNumber("GYRO_KP"));
    		if (kpDisabled || kpOff) {
    			newRotate2 = newRotate;
    		}
    		
        	SmartDashboard.putNumber("New Rotation 2", newRotate2);
        	m_drive.arcadeDrive(newMove, newRotate2);	
    	} else {
        	m_drive.arcadeDrive(newMove, newRotate);
    	}

    	// SET PREVIOUS VALUES
    	// ----------------------------------------------------------------------
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
    
    public boolean isSpeedMode() {
    	return this.speedMode;
    }
    
    public void toggleSpeedMode() {
    	this.speedMode = !speedMode;
    }
    
    public void setSpeedMode(boolean speedMode){
    	this.speedMode = speedMode;
    }
    
    public double curveDrive(double value, double prevValue, boolean isZeroStopped, double accelCurve, boolean isRotate){
        //velocity curve
        value = uCurve(value);
        
        //accel curve
        if(value == 0 && isZeroStopped)
            value = 0;
        else
            value = curve(value, prevValue, accelCurve);
        value = Util.limit(value, -1, 1);
        
        if (isRotate){
        	if (speedMode)
                return value / Constants.ROTATE_FAST_REDUCTION_FACTOR;
            else
                return value / Constants.ROTATE_SLOW_REDUCTION_FACTOR;
        } else {
        	if (speedMode)
                return value / Constants.DRIVE_FAST_REDUCTION_FACTOR;
            else
                return value / Constants.DRIVE_SLOW_REDUCTION_FACTOR;
        }
    }

	public void stop() {
		m_drive.arcadeDrive(0, 0);
	}
}