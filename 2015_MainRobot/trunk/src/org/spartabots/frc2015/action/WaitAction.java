package org.spartabots.frc2015.action;

/**
 * Waits, will only work in a SeriesAction. Won't do anything when added to a Profile
 */
public class WaitAction extends Action {
	
	public WaitAction(double milliseconds) {
		this.setTimeout(milliseconds);
	}

	@Override
	public void init() {
		// No code necessary
	}
	
	@Override
	public boolean runPeriodic() {
		return true; // time out handled by super class
	}

	@Override
	public void done() {
		// No code necessary
	}

}
