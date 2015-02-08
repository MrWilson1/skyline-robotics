package org.spartabots.frc2015.action;

public class ClampAction extends Action {
	public static int IN = 0;
	public static int OUT = 1;
	public static int clampState = 0;
	
	public ClampAction(int state) {
		ClampAction.clampState = state;
		if (state != 0 && state != 1)
			throw new IllegalArgumentException();
	}
	
	@Override
	public void init() {
		if (clampState == IN)
			robot.elevator.clampIn();
		else
			robot.elevator.clampOut();
		cancel();
	}

	@Override
	public boolean runPeriodic() {
		return false;
	}

	@Override
	public void done() {
		// Nothing to do here
	}

}
