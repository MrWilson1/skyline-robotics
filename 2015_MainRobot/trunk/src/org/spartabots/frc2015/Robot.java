package org.spartabots.frc2015;

import org.spartabots.frc2015.controller.XboxController;
import org.spartabots.frc2015.subsystem.Drive;

import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Victor;

/**
 * Main Robot class for initialization
 */
public class Robot extends SampleRobot {
    private static Robot instance;
    RobotDrive m_drive;
    XboxController driveController;
    Victor traverse;
    Drive drive;
    
    AutoMode autoMode;
    ControlMode controlMode;
    
    public Robot() {
    	super();
    	instance = this;
    	initHardware();
    	initSoftware();
    }
    
    public void initHardware() {
    	m_drive = new RobotDrive(Ports.DigitalSidecar.Pwm2, Ports.DigitalSidecar.Pwm1);
        m_drive.setExpiration(0.1);
        traverse = new Victor(Ports.DigitalSidecar.Pwm3);
    }

    public void initSoftware() {
        drive = new Drive(m_drive, false);
        driveController = new XboxController(Ports.Computer.Usb1);
        
        autoMode = new AutoMode(this);
        controlMode = new ControlMode(this);
    }
    
    public void autonomous() {
    	autoMode.run();
    }
    
    public void operatorControl() {
    	controlMode.run();
    }
    
    public void test() {
    }
    
    public static Robot getInstance() {
    	return instance;
    }
}
