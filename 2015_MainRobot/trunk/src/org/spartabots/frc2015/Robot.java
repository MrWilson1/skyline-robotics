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
    public int selectedAutoAction = 5;
    public XboxController driveController;
    public XboxController driveController2;
    
    // Subsystems
    public Drive drive;
    public Elevator elevator;
    
    // Profiles
    ControlProfile controlProfile;
    AutoProfile autoProfile;
    TestProfile testProfile;
    
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
    	this.driveController2 = new XboxController(Ports.Computer.Usb1);
    	
    	this.controlProfile = new ControlProfile();
    	this.autoProfile = new AutoProfile();
    	this.testProfile = new TestProfile();
    	
    	server = CameraServer.getInstance();
        server.setQuality(50);
        server.startAutomaticCapture("cam0");
    }
    
    @Override
    public void autonomous() {
    	autoProfile.setSelectedAction(selectedAutoAction);
    	autoProfile.start();;
    }

    @Override
    public void operatorControl() {
    	controlProfile.start();
    }

    @Override
    public void test() {
    	testProfile.start();
    }
    
    public static Robot getInstance() {
    	return instance;
    }
}
