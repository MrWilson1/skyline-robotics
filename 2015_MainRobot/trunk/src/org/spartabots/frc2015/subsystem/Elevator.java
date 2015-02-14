package org.spartabots.frc2015.subsystem;

import org.spartabots.frc2015.action.ClampAction;
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
    
    //Limit Switch
    public DigitalInput bottom_switch;
    
    // Misc state
    boolean elevatorMoving = false;
	public int clampState = 0;
	boolean speedMode = false;
	
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
		if (value != 0) {
			if (value < 0 && this.isAtBottom()) {
				if (this.isAtBottom()) {
					this.elevatorMoving = false;
					return;
				} else if (!speedMode) {
					value /= Constants.ELEVATOR_DOWN_REDUCTION_FACTOR;
				}
			} else if (value > 0 && !speedMode){
				value /= Constants.ELEVATOR_UP_REDUCTION_FACTOR;
			}
			this.elevatorMoving = true;
		} else {
			this.elevatorMoving = false;
		}
		
		this.eMotor.set(Util.ease(value, Constants.REGULAR_EASE_CONSTANT));
	}
	
	/* CLAMP
	 * -------------------------------------------------------------------------------- */
	
	public void clampIn() {
		gripSolenoid.set(false);
		this.clampState = ClampAction.IN;
	}
	
	public void clampOut() {
		gripSolenoid.set(true);
		this.clampState = ClampAction.OUT;
	}
	
	public boolean isAtBottom() {
		return bottom_switch.get();
	}

	public void toggleSpeedMode() {
		this.speedMode = !speedMode;
	}
}
