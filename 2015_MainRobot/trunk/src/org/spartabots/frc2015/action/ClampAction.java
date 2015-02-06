package org.spartabots.frc2015.action;

public class ClampAction extends Action {
	public static int IN = 0;
	public static int OUT = 1;
	int state = 0;
	
	public ClampAction(int state) {
		this.state = state;
		if (state != 0 || state != 1)
			throw new IllegalArgumentException();
	}
	
	@Override
	public void init() {
		if (state == IN)
			robot.elevator.clampIn();
		else
			robot.elevator.clampOut();
		done();
	}

	@Override
	public boolean running() {
		return false;
	}

	@Override
	public void done() {
		// Nothing to do here
	}

}
