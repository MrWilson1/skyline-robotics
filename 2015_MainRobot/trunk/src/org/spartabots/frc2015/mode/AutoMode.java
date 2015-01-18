package org.spartabots.frc2015.mode;

import org.spartabots.frc2015.Robot;

public class AutoMode extends Mode implements Runnable {

	public AutoMode(Robot robot) {
		super(robot, Mode.AUTONOMOUS);
	}

	@Override
	public void autoPeriodic() {
		
	}

}
