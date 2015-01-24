package org.spartabots.frc2015.action;

/**
 * Drives for a certain distance or for a certain time.
 * <br/><br/>
 * TODO time works, distance doesn't
 */
public class DriveAction extends Action {
	public static final int DISTANCE = 0;
	public static final int TIME = 0;
	int type;
	double dist;
	
	/**
	 * Drive a certain distance or for a certain time.
	 * 
	 * @param type DriveAction.DISTANCE or DriveAction.TIME
	 * @param value Distance in meters or time in milliseconds
	 */
	public DriveAction(int type, double value) {
		this.type = type;
		if (this.type == TIME)
			this.setTimeout(value);
		else if (this.type == DISTANCE)
			this.dist = value;
	}
	
	@Override
	public void init() {
		robot.drive.resetEncoders();
		robot.drive.drive(0.7, 0);
	}

	@Override
	public boolean running() {
		if (this.type == DISTANCE) {
			if (robot.drive.getLeftEncoderDistance() >= dist) {
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
