package org.spartabots.frc2015.action;


public class ResetAction extends Action {
	public static final int GYRO = 0;
	public static final int BELT_EC = 1;
	public static final int TRAVERSE_EC = 2;
	public static final int ZERO_EC = 3;
	public static final int ZERO_GYRO = 4;
	int[] values;
	
	public ResetAction(int... value) {
		this.values = value;
	}

	@Override
	public void init() {
		for (int val : values) {
			switch (val) {
			case GYRO:
				robot.drive.resetGyro();
				break;
			case BELT_EC:
				robot.drive.beltEc.reset();
				break;
			case TRAVERSE_EC:
				robot.drive.traverseEc.reset();
				break;
			case ZERO_EC:
				robot.drive.setZeroEcDist();
				break;
			case ZERO_GYRO:
				robot.drive.setZeroHeading();
				break;
			default:
				break;
			}
		}
		cancel();
	}

	@Override
	public boolean runPeriodic() {
		return false;
	}

	@Override
	public void done() {
	}

}
