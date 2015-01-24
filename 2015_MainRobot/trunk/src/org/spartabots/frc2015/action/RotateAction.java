package org.spartabots.frc2015.action;

import org.spartabots.frc2015.util.Util;

public class RotateAction extends Action {
	double heading;
	double headingError = 3;
	boolean headingCorrection = true;
	public static final int DEGREES = 0;
	public static final int RADIANS = 0;
	int unit = 0;
	
	public RotateAction(double heading, int unit) {
		this.heading = heading;
		this.unit = unit;
	}
	
	@Override
	public void init() {
		robot.drive.resetGyro();
		
		if (unit == DEGREES) {
			if (robot.drive.getGyroAngle() < (heading-headingError)) {
				robot.drive.drive(0, 0.6);
			} else if (robot.drive.getGyroAngle() > (heading+headingError)) {
				robot.drive.drive(0, -0.6);
			}
		} else {
			if (robot.drive.getGyroAngleRad() < (heading-headingError)) {
				robot.drive.drive(0, 0.6);
			} else if (robot.drive.getGyroAngleRad() > (heading+headingError)) {
				robot.drive.drive(0, -0.6);
			}
		}
	}
	
	@Override
	public boolean running() {
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
	}
	
}
