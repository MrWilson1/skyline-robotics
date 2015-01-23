package org.spartabots.frc2015;

import org.spartabots.frc2015.controller.XboxController;
import org.spartabots.frc2015.mode.AutoMode;
import org.spartabots.frc2015.mode.ControlMode;
import org.spartabots.frc2015.subsystem.Drive;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;

/**
 * Main Robot class for initialization
 */
public class Robot extends SampleRobot {
    private static Robot instance;
    
    // Drive
    public RobotDrive m_drive;
    public Drive drive;
    
    // Misc Motors
    public Talon traverse;
    public Talon elevator1;
    public Talon elevator2;
    
    // Pistons
    public Compressor compressor;
    public Solenoid eGrip1;
    public Solenoid eGrip2;
    
    // Modes
    public AutoMode autoMode;
    public ControlMode controlMode;
    
    // Controller
    public XboxController driveController;
    
    // Gyro
    Gyro gyro;
    double kp = 0.03;
    
    public Robot() {
    	super();
    	instance = this;
    	initHardware();
    	initSoftware();
    }
    
    public void initHardware() {
    	m_drive = new RobotDrive(
    			Ports.DigitalSidecar.Pwm2, // Front Left
    			Ports.DigitalSidecar.Pwm3, // Rear Left
    			Ports.DigitalSidecar.Pwm0, // Front Right
    			Ports.DigitalSidecar.Pwm1  // Rear Right
    			);
        m_drive.setExpiration(0.1);
        traverse = new Talon(Ports.DigitalSidecar.Pwm4);
        elevator1 = new Talon(Ports.DigitalSidecar.Pwm5);
        elevator2 = new Talon(Ports.DigitalSidecar.Pwm6);
        gyro = new Gyro(Ports.Crio.AnalogChannel11); // Dummy port
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
