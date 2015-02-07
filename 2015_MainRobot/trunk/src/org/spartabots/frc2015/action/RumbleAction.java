package org.spartabots.frc2015.action;

import edu.wpi.first.wpilibj.Joystick.RumbleType;

public class RumbleAction extends Action {
	public static final int RUMBLE_LEFT = 0;
	public static final int RUMBLE_RIGHT = 1;
	public static final int RUMBLE_BOTH = 2;
	final int rumbleType;
	final float value;
	
	/**
	 * Rumble the drive controller for a certain amount of time.
	 * 
	 * @param rumbleType
	 * @param value Rumble power
	 * @param timeMillis How long to rumble for (caps at 10 seconds)
	 */
	public RumbleAction(int rumbleType, float value, double timeMillis) {
		if (timeMillis > 10000) {
			timeMillis = 10000;
		}
		this.setTimeout(timeMillis);
		this.value = value;
		this.rumbleType = rumbleType;
	}
	
	@Override
	public void init() {
	}

	@Override
	public boolean runPeriodic() {
		if (rumbleType == RUMBLE_LEFT) {
			robot.driveController.setRumble(RumbleType.kLeftRumble, value);	
		} else if (rumbleType == RUMBLE_RIGHT) {
			robot.driveController.setRumble(RumbleType.kRightRumble, value);
		} else if (rumbleType == RUMBLE_BOTH) {
			robot.driveController.setRumble(RumbleType.kLeftRumble, value);
			robot.driveController.setRumble(RumbleType.kRightRumble, value);
		}
		
		return true;
	}

	@Override
	public void done() {
		if (rumbleType == RUMBLE_LEFT) {
			robot.driveController.setRumble(RumbleType.kLeftRumble, 0);	
		} else if (rumbleType == RUMBLE_RIGHT) {
			robot.driveController.setRumble(RumbleType.kRightRumble, 0);
		} else if (rumbleType == RUMBLE_BOTH) {
			robot.driveController.setRumble(RumbleType.kLeftRumble, 0);
			robot.driveController.setRumble(RumbleType.kRightRumble, 0);
		}
	}

}
