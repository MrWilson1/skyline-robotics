package org.spartabots.frc2015.subsystem;

import org.spartabots.frc2015.util.Constants;
import org.spartabots.frc2015.util.Util;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;

public class Elevator extends Subsystem {
	// Elevator Motors
    public Talon e1;
    public Talon e2;
    
    // Pistons
    public Compressor compressor;
    public Solenoid eGrip1;
    public Solenoid eGrip2;
    
    public Elevator() {
    	super();
    }
    
	protected void init() {
        compressor = new Compressor(Constants.COMPRESSOR_PORT);
        
        e1 = new Talon(Constants.ELEVATOR1_PORT);
        e2 = new Talon(Constants.ELEVATOR2_PORT);
	}
	
	public void setE1(double value) {
		e1.set(Util.ease(value, Constants.REGULAR_EASE_CONSTANT));
	}
	
	public void setE2(double value) {
		e2.set(Util.ease(value, Constants.REGULAR_EASE_CONSTANT));
	}
	
	/* CLAMP
	 * -------------------------------------------------------------------------------- */
	
	public void clampIn(){
		
	}
	
	public void clampOut(){
		
	}
	
	public void elevator(double move){
		
	}
}
