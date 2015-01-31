package org.spartabots.frc2015;

import org.spartabots.frc2015.profile.AutoProfile1;
import org.spartabots.frc2015.profile.AutoProfile2;
import org.spartabots.frc2015.profile.AutoProfile3;
import org.spartabots.frc2015.profile.AutoProfile4;
import org.spartabots.frc2015.profile.AutoProfileDoNothing;
import org.spartabots.frc2015.profile.ControlProfile;
import org.spartabots.frc2015.profile.Profile;
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
     * auto 1 is grab 1 tote and go accross
     * auto 2 grab 2 totes and go across
     * auto 3 is grab all 3 totes and go across
     * auto 4 is grab a bin and go across
     */
    public Profile autoProfile1 		= new AutoProfile1(this);
    public Profile autoProfile2 		= new AutoProfile2(this);
    public Profile autoProfile3 		= new AutoProfile3(this);
    public Profile autoProfile4 		= new AutoProfile4(this);
    public Profile autoProfile5 		= new AutoProfileDoNothing(this);
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
