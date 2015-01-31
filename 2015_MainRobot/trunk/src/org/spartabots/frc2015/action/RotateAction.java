package org.spartabots.frc2015.action;

import org.spartabots.frc2015.util.Util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RotateAction extends Action {
	double heading;
	double headingError = 3;
	boolean headingCorrection = true;
	public static final int DEGREES = 0;
	public static final int RADIANS = 0;
	int unit = 0;
	double dir = 0;
	
	public RotateAction(double heading, int unit) {
		this.heading = heading;
		this.unit = unit;
	}
	
	@Override
	public void init() {
		robot.drive.resetGyro();
		
		if (unit == DEGREES) {
			SmartDashboard.putString("Rotate Init", robot.drive.getGyroAngle() + "; " + (heading-headingError) + ", " + (heading+headingError));
			if (robot.drive.getGyroAngle() < (heading-headingError)) {
				this.dir = 0.5;
				SmartDashboard.putString("Rotate Init Dir", "Positive");
			} else if (robot.drive.getGyroAngle() > (heading+headingError)) {
				this.dir = -0.5;
				SmartDashboard.putString("Rotate Init Dir", "Negative");
			} else {
				SmartDashboard.putString("Rotate Init Dir", "Not chosen");
			}
		} else {
			if (robot.drive.getGyroAngleRad() < (heading-headingError)) {
				robot.drive.m_drive.arcadeDrive(0, 0.5);
			} else if (robot.drive.getGyroAngleRad() > (heading+headingError)) {
				robot.drive.m_drive.arcadeDrive(0, -0.5);
			}
		}
	}
	
	@Override
	public boolean running() {
        SmartDashboard.putNumber("Gyro Angle", robot.drive.getGyroAngle());
        SmartDashboard.putNumber("Acceleration X", robot.drive.getAccelX());
        SmartDashboard.putNumber("Acceleration Y", robot.drive.getAccelY());
        SmartDashboard.putNumber("Acceleration Z", robot.drive.getAccelZ());
        SmartDashboard.putNumber("Right Encoder Distance", robot.drive.getRightEncoderDistance());
        SmartDashboard.putNumber("Left Encoder Distance", robot.drive.getLeftEncoderDistance());

		robot.drive.m_drive.arcadeDrive(0.1, dir);
		
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
		robot.drive.resetGyro();
        SmartDashboard.putNumber("Gyro Angle", robot.drive.getGyroAngle());
	}
	
}
