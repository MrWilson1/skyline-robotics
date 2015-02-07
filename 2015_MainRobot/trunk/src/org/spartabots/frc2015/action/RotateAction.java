package org.spartabots.frc2015.action;

import org.spartabots.frc2015.util.Util;

public class RotateAction extends Action {
	double heading;
	double headingError = 10;
	public static final int DEGREES = 0;
	public static final int RADIANS = 1;
	int unit = 0;
	double speed = 0;
	
	public RotateAction(double heading, int unit, double speed) {
		if (unit != DEGREES || unit != RADIANS || speed > 1 || speed < 0)
			throw new IllegalArgumentException();
		this.heading = heading;
		this.unit = unit;
	}
	
	@Override
	public void init() {
		robot.drive.setZeroHeading();
		
		if (unit == DEGREES) {
			if (robot.drive.getGyroAngle() < (heading-headingError)) {
				this.speed = +speed;
			} else if (robot.drive.getGyroAngle() > (heading+headingError)) {
				this.speed = -speed;
			}
		} else {
			if (robot.drive.getGyroAngleRad() < (heading-headingError)) {
				this.speed = +speed;
			} else if (robot.drive.getGyroAngleRad() > (heading+headingError)) {
				this.speed = -speed;
			}
		}
	}
	
	@Override
	public boolean runPeriodic() {
		robot.drive.m_drive.arcadeDrive(0, speed);
		
		if (unit == DEGREES) {
			if (Util.isWithinRange(robot.drive.getGyroAngle(), heading, headingError)) {
				return false;
			}
		} else {
			if (Util.isWithinRange(robot.drive.getGyroAngleRad(), heading, headingError)) {
				return false;
			}
		}
		return true;
	}
	
	@Override
	public void done() {
		robot.drive.stop();
		robot.drive.setZeroHeading();
	}
	
}
