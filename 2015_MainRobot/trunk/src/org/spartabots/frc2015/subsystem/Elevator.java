package org.spartabots.frc2015.subsystem;

import org.spartabots.frc2015.action.ClampAction;
import org.spartabots.frc2015.util.Constants;
import org.spartabots.frc2015.util.Util;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;

// TODO Elevator encoders
public class Elevator extends Subsystem {
	// Elevator Motors
    public Talon eMotor; 
    
    // Pistons
    public Compressor compressor;
    public Solenoid gripSolenoid;
    
    // Limit Switch
    public DigitalInput bottom_switch;
    public DigitalInput top_switch;
    
    // Misc state
    public boolean elevatorMoving = false;
	public int clampState = ClampAction.IN;
	public boolean speedMode = false;
	public Encoder ec;
	
    public Elevator() {
    	super();
    }
    
	protected void init() {
        compressor 		= new Compressor(Constants.COMPRESSOR_PORT);
        
        eMotor 			= new Talon(Constants.ELEVATOR_PORT);
        
        bottom_switch	= new DigitalInput(Constants.ELEVATOR_BOTTOM);
        top_switch		= new DigitalInput(Constants.ELEVATOR_TOP);
	
        gripSolenoid 	= new Solenoid(Constants.SOLENOID_EGRIP);
        

       ec = new Encoder(Constants.ELEVATOR_EC_A, Constants.ELEVATOR_EC_B, false);
	}
	
	public void setElevator(double value) {
		if (value != 0) {
			if (value < 0) {
				if (this.isAtBottom()) {
					this.elevatorMoving = false;
					this.eMotor.set(0);
					return;
				} else if (!speedMode) {
					value /= Constants.ELEVATOR_DOWN_REDUCTION_FACTOR;
				}
			} else if (value > 0){
				if (this.isAtTop()) {
					this.elevatorMoving = false;
					this.eMotor.set(0);
					return;
				} else if (!speedMode) {
					value /= Constants.ELEVATOR_UP_REDUCTION_FACTOR;	
				}
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
	
	public boolean isAtTop() {
		return top_switch.get();
	}

	public void toggleSpeedMode() {
		this.speedMode = !speedMode;
	}
	
	public boolean isSpeedMode() {
		return this.speedMode;
	}
	
	public double getDistanceFromGround() {
		return ec.getRaw() * Constants.ELEVATOR_EC_ENCODER_TO_FEET_RATIO + 0.5;
	}
}
