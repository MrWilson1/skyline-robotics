package org.spartabots.frc2015.action;

import org.spartabots.frc2015.subsystem.Elevator;


public class ReleaseAction extends Action {
	
	
	@Override
	public void init() {
		// if already clamp out, do nothing
		// else if elevator at bottom, then just clamp out
		// else RealReleaseAction
		
		//if clamp out
		if(ClampAction.clampState == 1){
			
		}else if(Elevator.bottom_switch.equals(0)){
			Actions.clampOut();
		}else{
			Actions.elevatorTime(-0.5, 1500);
			Actions.clampOut();
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
	
	static class RealReleaseAction extends SeriesAction {
		@Override
		public void init() {
			enqueue(Actions.elevatorTime(-0.5, 1500));
			enqueue(Actions.clampOut());
		}
	}
}
