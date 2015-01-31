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
	
        eGrip1 = new Solenoid(Constants.SOLENOID_EGRIP1);
        eGrip2 = new Solenoid(Constants.SOLENOID_EGRIP2);
	}
	
	public void setE1(double value) {
		e1.set(Util.ease(value, Constants.REGULAR_EASE_CONSTANT));
	}
	
	public void setE2(double value) {
		e2.set(Util.ease(value, Constants.REGULAR_EASE_CONSTANT));
	}
	
	/* CLAMP
	 * -------------------------------------------------------------------------------- */
	
	public void clampE1In(){
		eGrip1.set(false);
	}
	
	public void clampE1Out(){
		eGrip1.set(true);
	}
	
	public void clampE2In(){
		eGrip2.set(false);
	}
	
	public void clampE2Out(){
		eGrip2.set(true);
	}
}
