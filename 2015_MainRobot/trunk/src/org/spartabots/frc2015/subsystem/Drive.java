package org.spartabots.frc2015.subsystem;

import edu.wpi.first.wpilibj.RobotDrive;

public class Drive {
    RobotDrive m_drive;
    double prevY;
    double prevX;
    boolean prevBDown;
    boolean speed;
    
    public Drive(RobotDrive m_drive, boolean speedmode){
        this.m_drive = m_drive;
        prevY = 0;
        prevX = 0;
        prevBDown = false;
        speed = speedmode;
    }
    
    /**
     * Don't try this
     * 
     * @param leftY
     * @param rightX
     */
    public void move(double leftY, double rightX){
        m_drive.arcadeDrive(curveDrive(rightX, prevX, false, 2), curveDrive(leftY, prevY, true, 3));
            
        // Set previous values for next loop
        prevX = rightX;
        prevY = leftY;
    }
    
    private double curve(double value, double prevValue, double accelCurve){
        return prevValue + (value - prevValue) / accelCurve;
    }
    
    private double cap(double value){
        if(value > 1)
            return 1;
        if (value < -1)
            return -1;
        else 
            return value;
    }
    
    private double UCurve(double value){
        if(value < 0)
            return -(value * value);
        else
            return value * value;
    }
    
    public void reset(){
        prevY = 0;
        prevX = 0;        
    }
    
    public void speedmode(boolean bButton){
        if (bButton){
            if(!prevBDown)
                speed = !speed;
            prevBDown = true;
        }
        else
            prevBDown = false;
    }
    public double curveDrive(double value, double prevValue, boolean isZeroStopped, double accelCurve){
        //velocity curve
        value = UCurve(value);
        
        //accel curve
        if(value == 0 && isZeroStopped)
            value = 0;
        else
            value = curve(value, prevValue, accelCurve);
        
        //caps at 1, -1
        value = cap(value);
        
        if (speed)
            return value;
        else
            return value / 1.5;
    }
}