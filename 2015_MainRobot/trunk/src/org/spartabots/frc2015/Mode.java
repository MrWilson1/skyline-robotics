package org.spartabots.frc2015;

import edu.wpi.first.wpilibj.Timer;

public abstract class Mode implements Runnable {
	Robot robot;
	private int mode;
	public static final int CONTROL = 0;
	public static final int AUTONOMOUS = 1;
	
	public Mode(Robot robot, int mode) {
		this.robot = robot;
		this.mode = mode;
	}
	
	@Override
	public void run() {
		if (mode == CONTROL) {
			while (robot.isOperatorControl() && robot.isEnabled()) {
				this.controlPeriodic();
				Timer.delay(0.005);
			}
		} else if (mode == AUTONOMOUS) {
			while (robot.isAutonomous() && robot.isEnabled()) {
				this.autoPeriodic();
				Timer.delay(0.005);
			}
		}
	}

	public void controlPeriodic() {
	}

	public void autoPeriodic() {
	}
}
