package org.spartabots.frc2015.subsystem;

import org.spartabots.frc2015.Ports;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;

public class Elevator extends Subsystem {
	// Elevator Motors
    public Talon elevator1;
    public Talon elevator2;
    
    // Pistons
    public Compressor compressor;
    public Solenoid eGrip1;
    public Solenoid eGrip2;
    
    public Elevator() {
    	super();
    }
    
	protected void init() {
        compressor = new Compressor(0);
        
        elevator1 = new Talon(Ports.RoboRIO.Pwm3);
        elevator2 = new Talon(Ports.RoboRIO.Pwm4);
	}
}
