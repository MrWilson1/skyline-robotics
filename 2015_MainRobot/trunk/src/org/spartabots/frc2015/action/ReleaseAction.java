package org.spartabots.frc2015.action;

public class ReleaseAction extends Action {
	
	@Override
	public void init() {
		// TODO Finish this
		// if already clamp out, do nothing
		// else if elevator at bottom, then just clamp out
		// else RealReleaseAction
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
	
	static class RealReleaseAction extends SeriesAction {
		@Override
		public void init() {
			enqueue(Actions.elevatorTime(-0.5, 1500));
			enqueue(Actions.clampOut());
		}
	}
}
