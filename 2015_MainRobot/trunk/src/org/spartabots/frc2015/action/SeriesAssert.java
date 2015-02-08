package org.spartabots.frc2015.action;

public abstract class SeriesAssert extends Action {
	
	public abstract boolean check();
	
	@Override
	public final void init() {
		// Nothing to do here
	}

	@Override
	public final boolean runPeriodic() {
		// Nothing to do here
		return false;
	}

	@Override
	public final void done() {
		// Nothing to do here
		
	}

}
