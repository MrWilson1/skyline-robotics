package org.spartabots.frc2015.subsystem;

import org.spartabots.frc2015.Robot;

public abstract class Subsystem {
	Robot robot = Robot.getInstance();
	
	public Subsystem() {
		init();
	}
	
	protected abstract void init();
}
