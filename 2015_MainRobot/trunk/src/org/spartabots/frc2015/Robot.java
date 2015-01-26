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
    public Drive drive = new Drive();
    public Elevator elevator = new Elevator();
    
    // Profiles
    public Profile autoProfile1 = new AutoProfile1();
    public Profile autoProfile2 = new AutoProfile2();
    public Profile autoProfile3 = new AutoProfile3();
    public Profile autoProfile4 = new AutoProfile4();
    public Profile autoProfile5 = new AutoProfileDoNothing();
    public Profile controlProfile = new ControlProfile();
    public static int selectedAutoProfile = 1;
    
    public Robot() {
    	super();
    	instance = this;
    }
    
    @Override
    public void autonomous() {
    	switch (selectedAutoProfile) {
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
