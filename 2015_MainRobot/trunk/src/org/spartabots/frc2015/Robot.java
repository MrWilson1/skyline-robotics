package org.spartabots.frc2015;

import org.spartabots.frc2015.profile.*;
import org.spartabots.frc2015.subsystem.Drive;
import org.spartabots.frc2015.subsystem.Elevator;
import org.spartabots.frc2015.util.Constants;
import org.spartabots.frc2015.util.XboxController;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Main Robot class for initialization
 */
public class Robot extends SampleRobot {
    private static Robot instance;
    public XboxController driveController;
    public XboxController loadController;
    
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
    
    // Sendable Chooser
    SendableChooser autoChooser;
    
    public Robot() {
    	super();
    	instance = this;
    }
    
    @Override
    public void robotInit() {
    	this.drive = new Drive();
    	this.elevator = new Elevator();
    	this.driveController = new XboxController(Ports.Computer.Usb0);
    	this.loadController = new XboxController(Ports.Computer.Usb1);
    	
    	elevator.ec.reset();
    	
    	controlProfile = new ControlProfile();
    	autoProfile = new AutoProfile();
    	testProfile = new TestProfile();
    	
    	server = CameraServer.getInstance();
        server.setQuality(50);
        server.setQuality(50);
        server.startAutomaticCapture("cam0");
        
        autoChooser = new SendableChooser();
        autoChooser.addDefault("Do nothing", 0);
        autoChooser.addObject("1 Tote", 1);
        autoChooser.addObject("2 Tote", 2);
        autoChooser.addObject("3 Tote", 3);
        autoChooser.addObject("1 Bin", 4);
        autoChooser.addObject("Drive only", 5);
        autoChooser.addObject("Testing: rotate 90 & elevator", 6);
        autoChooser.addObject("Testing: repickup", 7);
        SmartDashboard.putData("Autonomous Mode Chooser", autoChooser);
        
        SmartDashboard.putNumber("GYRO_KP", Constants.GYRO_KP);
    }
    
    @Override
    public void autonomous() {
    	autoProfile.setSelectedAction((int) autoChooser.getSelected());
    	autoProfile.start();
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
