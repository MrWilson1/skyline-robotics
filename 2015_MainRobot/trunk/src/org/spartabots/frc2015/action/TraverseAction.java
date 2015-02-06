package org.spartabots.frc2015.action;

public class TraverseAction extends DriveAction {

	public TraverseAction(int type, double value, double speed) {
		super(type, value, speed);
		if (type == TraverseAction.DISTANCE)
			throw new UnsupportedOperationException("Traversing a certain distance is not yet implemented (encoder nonexistant)");
	}
	
	@Override
	protected void drive() {
		robot.drive.traverse.set(speed);
	}

	@Override
	protected double getEncoderDistance() {
		return 0; // TODO no encoder exists yet for traverse wheel
	}
}
