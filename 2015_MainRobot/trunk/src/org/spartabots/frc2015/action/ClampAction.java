package org.spartabots.frc2015.action;

public class ClampAction extends Action {
	public static int IN = 0;
	public static int OUT = 1;
	public int new_state = 0;
	
	public ClampAction(int state) {
		if (state != 0 && state != 1)
			throw new IllegalArgumentException();
		this.new_state = state;
	}
	
	@Override
	public void init() {
		if (new_state == IN)
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
