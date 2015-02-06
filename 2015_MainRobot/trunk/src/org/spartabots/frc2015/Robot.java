package org.spartabots.frc2015;

import org.spartabots.frc2015.profile.*;
import org.spartabots.frc2015.subsystem.Drive;
import org.spartabots.frc2015.subsystem.Elevator;
import org.spartabots.frc2015.util.XboxController;

import edu.wpi.first.wpilibj.SampleRobot;

/**
 * Main Robot class for initialization
 */
public class Robot extends SampleRobot {
    private static Robot instance;
    public int selectedAutoProfile = 6;
    public XboxController driveController;
    
    // Subsystems
    public Drive drive;
    public Elevator elevator;
    
    // Current profile
    public Profile profile;
    
    public Robot() {
    	super();
    	instance = this;
    }
    
    @Override
    public void robotInit() {
    	this.drive = new Drive();
    	this.elevator = new Elevator();
    	this.driveController = new XboxController(Ports.Computer.Usb0);
    }
    
    @Override
    public void autonomous() {
    	new AutoProfile(this, selectedAutoProfile).start();;
    }

    @Override
    public void operatorControl() {
    	new ControlProfile(this).start();
    }

    @Override
    public void test() {
    	new TestProfile(this).start();
    }
    
    public static Robot getInstance() {
    	return instance;
    }
}
