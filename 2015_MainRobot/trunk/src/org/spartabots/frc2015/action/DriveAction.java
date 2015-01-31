package org.spartabots.frc2015.action;

/**
 * Drives for a certain distance or for a certain time.
 */
public class DriveAction extends Action {
	public static final int DISTANCE = 0;
	public static final int TIME = 1;
	int type;
	double dist;
	double speed = 0.7;
	double error = 0.08;
	
	/**
	 * Drive a certain distance or for a certain time.
	 * 
	 * @param type DriveAction.DISTANCE or DriveAction.TIME
	 * @param value Distance in meters or time in milliseconds
	 * @param speed How fast to move (0 to 1)
	 */
	public DriveAction(int type, double value, double speed) {
		this.type = type;
		if (value < 0 || speed < 0)
			throw new IllegalArgumentException();
		if (this.type == TIME)
			this.setTimeout(value);
		else if (this.type == DISTANCE)
			this.dist = value;
		else
			throw new IllegalArgumentException();
		this.speed = speed;
	}
	
	@Override
	public void init() {
		robot.drive.resetEncoders();
	}

	@Override
	public boolean running() {
		robot.drive.drive(speed, 0);
		if (this.type == DISTANCE) {
			if (robot.drive.getLeftEncoderDistance() >= (dist-error)) {
				return false;
			}
		}
		return true;
	}

	@Override
	public void done() {
		robot.drive.stop();
		robot.drive.resetEncoders();
	}

}
