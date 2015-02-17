package org.spartabots.frc2015.action;

import org.spartabots.frc2015.util.Util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RotateAction extends Action {
	double heading;
	double headingError = 10;
	public static final int DEGREES = 0;
	public static final int RADIANS = 1;
	int unit = 0;
	double speed = 0;
	
	public RotateAction(double heading, int unit, double speed) {
		if (unit != DEGREES && unit != RADIANS)
			throw new IllegalArgumentException();
		if (1 < speed || speed < 0)
			throw new IllegalArgumentException();
		this.heading = heading;
		this.unit = unit;
		this.speed = speed;
	}
	
	@Override
	public void init() {
		robot.drive.setZeroHeading();
		robot.drive.lockGyroOffsetChange = true;
		
		SmartDashboard.putString("Whatcha doing rotate?", "init");
		
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

		SmartDashboard.putString("Whatcha doing rotate?", "init finish");
		SmartDashboard.putString("rotate init", heading + ", " + speed);
	}
	
	@Override
	public boolean runPeriodic() {
		robot.drive.m_drive.arcadeDrive(0, speed);

		SmartDashboard.putString("Whatcha doing rotate?", "rotating, speed=" + speed + ", to="+heading);
		
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
		robot.drive.lockGyroOffsetChange = false;
		robot.drive.stop();
		robot.drive.setZeroHeading();
	}
	
}
