package org.spartabots.frc2015.action;

public class TraverseAction extends DriveAction {

	public TraverseAction(int type, double value, double speed) {
		super(type, value, speed);
	}
	
	@Override
	public void init() {
		super.init();
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
		robot.drive.resetEncoders();
	}
}
