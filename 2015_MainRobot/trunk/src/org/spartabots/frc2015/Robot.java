package org.spartabots.frc2015;

import org.spartabots.frc2015.profile.*;
import org.spartabots.frc2015.subsystem.Drive;
import org.spartabots.frc2015.subsystem.Elevator;

import edu.wpi.first.wpilibj.SampleRobot;

/**
 * Main Robot class for initialization
 */
public class Robot extends SampleRobot {
    private static Robot instance;
    
    // Subsystems
    public Drive drive 					= new Drive();
    public Elevator elevator 			= new Elevator();
    
    /* Profiles
     * auto 1: grab 1 tote and go accross
     * auto 2: grab 2 totes and go across
     * auto 3: grab 3 totes and go across
     * auto 4: grab 1 bin and go across
     */
    public Profile autoProfile1 		= new Auto1Tote(this);
    public Profile autoProfile2 		= new Auto2Tote(this);
    public Profile autoProfile3 		= new Auto3Tote(this);
    public Profile autoProfile4 		= new Auto1Bin(this);
    public Profile autoProfile5 		= new AutoDoNothing(this);
    public Profile autoProfile6 		= new AutoDriveCenter(this);
    public Profile controlProfile	 	= new ControlProfile(this);
    public int selAutoProfile		 	= 1;
    
    public Robot() {
    	super();
    	instance = this;
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
        default:
        	autoProfile5.start();
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
