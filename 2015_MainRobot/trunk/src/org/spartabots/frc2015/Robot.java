package org.spartabots.frc2015;

import org.spartabots.frc2015.profile.*;
import org.spartabots.frc2015.subsystem.Drive;
import org.spartabots.frc2015.subsystem.Elevator;

import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.vision.AxisCamera;

/**
 * Main Robot class for initialization
 */
public class Robot extends SampleRobot {
    private static Robot instance;
    
    // Subsystems
    public Drive drive 					= new Drive();
    public Elevator elevator 			= new Elevator();

    // Profiles
    public Profile autoProfile1, autoProfile2, autoProfile3, autoProfile4, autoProfile5, autoProfile6, controlProfile;
    public int selAutoProfile = 6;
    
    // Camera
    AxisCamera cam;
    
    public Robot() {
    	super();
    	instance = this;
    }
    
    @Override
    public void robotInit() {
    	autoProfile1 	= new Auto1Tote(this); 			// Grab 1 tote, go across
    	autoProfile2 	= new Auto2Tote(this); 			// Grab 2 tote, go across
        autoProfile3 	= new Auto3Tote(this); 			// Grab 3 tote, go across
        autoProfile4 	= new Auto1Bin(this);  			// Grab 1 bin, go across
        autoProfile5 	= new AutoDoNothing(this); 		// Do nothing
        autoProfile6 	= new AutoDriveCenter(this); 	// Only drive to center
        controlProfile	= new ControlProfile(this); 	// Teleop control
        //cam 			= new AxisCamera("10.29.76.11");
    }
    
    @Override
    public void autonomous() {
    	switch (selAutoProfile) {
    	case 1:
        	autoProfile1.start();
        	break;
    	case 2:
        	autoProfile2.start();
        	break;
    	case 3:
        	autoProfile3.start();
        	break;
    	case 4:
        	autoProfile4.start();
        	break;
    	case 5:
    		autoProfile5.start();
    		break;
    	case 6:
    		autoProfile6.start();
    		break;
        default:
        	autoProfile6.start();
    	}
    }

    @Override
    public void operatorControl() {
    	controlProfile.start();
    }

    @Override
    public void test() {
    }
    
    public static Robot getInstance() {
    	return instance;
    }
}
