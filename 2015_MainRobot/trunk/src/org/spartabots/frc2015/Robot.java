package org.spartabots.frc2015;

import org.spartabots.frc2015.profile.*;
import org.spartabots.frc2015.subsystem.Drive;
import org.spartabots.frc2015.subsystem.Elevator;
import org.spartabots.frc2015.util.Constants;
import org.spartabots.frc2015.util.XboxController;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.Image;

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
    public int cam_session;
    public Image cam_frame;
    public Thread cameraThread;
    
    // Sendable Chooser
    public SendableChooser autoChooser;
    public SendableChooser autoRotateChooser;
    
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

        cam_frame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);
        cam_session = NIVision.IMAQdxOpenCamera("cam0",
                NIVision.IMAQdxCameraControlMode.CameraControlModeController);
        NIVision.IMAQdxConfigureGrab(cam_session);
        
        autoChooser = new SendableChooser();
        autoChooser.addDefault("Do nothing", 0);
        autoChooser.addObject("1 Tote", 1);
        autoChooser.addObject("2 Tote", 2);
        autoChooser.addObject("1 Tote Special", 3);
        autoChooser.addObject("1 Bin", 4);
        autoChooser.addObject("Drive only", 5);
        autoChooser.addObject("Testing: rotate 90 & elevator", 6);
        autoChooser.addObject("Testing: repickup", 7);
        SmartDashboard.putData("Autonomous Mode Chooser", autoChooser);
        
        autoRotateChooser = new SendableChooser();
        autoRotateChooser.addDefault("Auto Rotate Right", 1);
        autoRotateChooser.addObject("Auto Rotate Left", -1);
        SmartDashboard.putData("Autonomous Mode Rotate Chooser", autoRotateChooser);
        
        SmartDashboard.putNumber("GYRO_KP", Constants.GYRO_KP);
        
        this.cameraThread = new Thread() {
        	public void run() {
                NIVision.IMAQdxStartAcquisition(cam_session);
                
                while (Robot.isReal()) {
                    NIVision.IMAQdxGrab(cam_session, cam_frame, 1);
                    CameraServer.getInstance().setImage(cam_frame);
                }
                
                NIVision.IMAQdxStopAcquisition(cam_session);
        	}
        };
        this.cameraThread.start();
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
