package org.spartabots.frc2015.action;

public class ReleaseAction extends Action {
	
	@Override
	public void init() {
		if (robot.elevator.clampState == ClampAction.OUT) {
			// Do nothing
		} else if (robot.elevator.isAtBottom()) { // Already at bottom
			robot.profile.add(Actions.clampOut());
		} else {
			robot.profile.add(new WithElevatorReleaseAction());
		}	
		cancel();
	}

	@Override
	public boolean runPeriodic() {
		// Nothing to do here
		return false;
	}

	@Override
	public void done() {
		// Nothing to do here
	}
	
	static class WithElevatorReleaseAction extends SeriesAction {
		@Override
		public void init() {
			enqueue(Actions.elevatorTime(-0.5, 1500));
			enqueue(Actions.clampOut());
		}
	}
}
