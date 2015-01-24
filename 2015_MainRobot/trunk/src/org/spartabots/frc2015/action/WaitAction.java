package org.spartabots.frc2015.action;

public class WaitAction extends Action {
	
	public WaitAction(double milliseconds) {
		this.setTimeout(milliseconds);
	}

	@Override
	public void init() {
		// No code necessary
	}
	
	@Override
	public boolean running() {
		return true; // time out handled by super class
	}

	@Override
	public void done() {
		// No code necessary
	}

}
