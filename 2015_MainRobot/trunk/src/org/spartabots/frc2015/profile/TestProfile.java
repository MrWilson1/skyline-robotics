package org.spartabots.frc2015.profile;

public class TestProfile extends Profile {

	public TestProfile() {
		super(Profile.TEST);
	}
	
	@Override
	public void init() {
		robot.drive.resetGyro();
		robot.drive.resetEncoders();
	}
	
	@Override
	public void done() {
		
	}

}
