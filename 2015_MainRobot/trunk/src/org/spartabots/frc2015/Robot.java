package org.spartabots.frc2015;

import org.spartabots.frc2015.profile.*;
import org.spartabots.frc2015.subsystem.Drive;
import org.spartabots.frc2015.subsystem.Elevator;
import org.spartabots.frc2015.util.XboxController;

import edu.wpi.first.wpilibj.CameraServer;
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
    
    // Camera
    CameraServer server;
    
    public Robot() {
    	super();
    	instance = this;
    }
    
    @Override
    public void robotInit() {
    	this.drive = new Drive();
    	this.elevator = new Elevator();
    	this.driveController = new XboxController(Ports.Computer.Usb0);
    	
    	server = CameraServer.getInstance();
        server.setQuality(50);
        //the camera name (ex "cam0") can be found through the roborio web interface
        server.startAutomaticCapture("cam0");
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
