package org.spartabots.frc2015.action;

public class TraverseAction extends DriveAction {

	public TraverseAction(int type, double value, double speed, boolean resetEncoders) {
		super(type, value, speed, resetEncoders);
	}
	
	@Override
	public void init() {
		if (resetEncoders)
			robot.drive.resetEncoders();
	}
	
	@Override
	protected void drive() {
		robot.drive.traverse.set(speed);
	}

	@Override
	protected double getEncoderDistance() {
		return robot.drive.getTraverseEncoderDistance();
	}
	
	@Override
	public void done() {
		robot.drive.traverse.set(0);
		if (resetEncoders)
			robot.drive.resetEncoders();
	}
}
