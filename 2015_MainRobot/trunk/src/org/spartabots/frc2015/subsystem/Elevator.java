package org.spartabots.frc2015.subsystem;

import org.spartabots.frc2015.util.Constants;
import org.spartabots.frc2015.util.Util;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;

// TODO Elevator encoders
public class Elevator extends Subsystem {
	// Elevator Motors
    public Talon eMotor; 
    
    // Pistons
    public Compressor compressor;
    public Solenoid gripSolenoid;
    boolean elevatorMoving = false;
    
    //Limit Switch
    public static DigitalInput bottom_switch;
    
    public Elevator() {
    	super();
    }
    
	protected void init() {
        compressor = new Compressor(Constants.COMPRESSOR_PORT);
        
        eMotor = new Talon(Constants.ELEVATOR_PORT);
        
        bottom_switch  = new DigitalInput(Constants.ELEVATOR_BOTTOM);
	
        gripSolenoid = new Solenoid(Constants.SOLENOID_EGRIP);
	}
	
	public void setElevator(double value) {
		if (value != 0)
			elevatorMoving = true;
		else
			elevatorMoving = false;
		eMotor.set(Util.ease(value, Constants.REGULAR_EASE_CONSTANT));
	}
	
	/* CLAMP
	 * -------------------------------------------------------------------------------- */
	
	public void clampIn(){
		gripSolenoid.set(false);
	}
	
	public void clampOut(){
		gripSolenoid.set(true);
	}
}
