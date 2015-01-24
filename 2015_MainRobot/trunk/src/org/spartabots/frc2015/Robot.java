package org.spartabots.frc2015;

import org.spartabots.frc2015.mode.AutoMode;
import org.spartabots.frc2015.mode.ControlMode;
import org.spartabots.frc2015.subsystem.Drive;
import org.spartabots.frc2015.subsystem.Elevator;

import edu.wpi.first.wpilibj.SampleRobot;

/**
 * Main Robot class for initialization
 */
public class Robot extends SampleRobot {
    private static Robot instance;
    
    // Subsystems
    public Drive drive = new Drive();
    public Elevator elevator = new Elevator();
    
    // Modes
    public AutoMode autoMode = new AutoMode(this);
    public ControlMode controlMode = new ControlMode(this);;
    
    public Robot() {
    	super();
    	instance = this;
    }
    
    @Override
    public void autonomous() {
    	autoMode.start();
    }

    @Override
    public void operatorControl() {
    	controlMode.start();
    }

    @Override
    public void test() {
    }
    
    public static Robot getInstance() {
    	return instance;
    }
}
