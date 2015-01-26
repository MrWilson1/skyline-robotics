package org.spartabots.frc2015.profile;

import org.spartabots.frc2015.util.Constants;

public class AutoProfile1 extends Profile {

	public AutoProfile1() {
		super(Profile.AUTONOMOUS);
	}
	
	@Override
	public void init() {
		robot.drive.resetGyro();
	}

	@Override
	public void autoPeriodic() {
		double angle = robot.drive.getGyroAngle();
		robot.drive.m_drive.arcadeDrive(robot.drive.curveDrive(1, 1, true, 3), -angle * Constants.GYRO_KP);
	}

	
	@Override
	public void done() {
		
	}

}
