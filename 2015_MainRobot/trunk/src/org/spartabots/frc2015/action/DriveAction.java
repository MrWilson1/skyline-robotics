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
	int direction = 0; // 0 - forward, 1 = backward
	
	/**
	 * Drive a certain distance or for a certain time.
	 * 
	 * @param type DriveAction.DISTANCE or DriveAction.TIME
	 * @param value Distance in meters or time in milliseconds (negative drives opposite direction)
	 * @param speed How fast to move from 0 to 1 (non directional)
	 */
	public DriveAction(int type, double value, double speed) {
		this.type = type;
		if (speed < 0)
			throw new IllegalArgumentException();
		if (this.type == TIME) {
			if (value < 0) throw new IllegalArgumentException();
			this.setTimeout(value);
		} else if (this.type == DISTANCE) {
			if (value < 0) {
				this.direction = 1;
				speed = -speed;
			}
			if (value == 0) cancel();
			this.dist = value;
		} else {
			throw new IllegalArgumentException();
		}
		this.speed = speed;
		robot.drive.isDrivingStraight = true;
	}
	
	@Override
	public void init() {
		robot.drive.resetEncoders();
	}

	@Override
	public boolean runPeriodic() {
		drive();
		if (this.type == DISTANCE) {
			if (direction == 0) { // forward, positive encoder distance
				if (getEncoderDistance() >= (dist-error)) {
					return false;
				}
			} else { // backward, negative encoder distance
				if (getEncoderDistance() <= (dist+error)) {
					return false;
				}
			}
		}
		return true;
	}
	
	// For Actions that may extend this DriveAction, so running() does not have to be overriden
	protected void drive() {
		robot.drive.drive(speed, 0);
	}

	// For Actions that may extend this DriveAction, so running() does not have to be overriden
	protected double getEncoderDistance() {
		return robot.drive.getLeftEncoderDistance();
	}
	
	@Override
	public void done() {
		robot.drive.stop();
		robot.drive.resetEncoders();
	}

}
