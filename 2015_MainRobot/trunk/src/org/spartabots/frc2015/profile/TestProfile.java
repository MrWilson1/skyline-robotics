package org.spartabots.frc2015.profile;

import org.spartabots.frc2015.Robot;

public class TestProfile extends Profile {

	public TestProfile(Robot robot) {
		super(Profile.TEST, robot);
	}
	
	@Override
	public void init() {
		robot.drive.resetGyro();
		robot.drive.resetEncoders();
	}

	@Override
	public void testPeriodic() {
	}
	
	@Override
	public void done() {
		
	}

}
