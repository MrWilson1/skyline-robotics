package org.spartabots.frc2015.profile;

import org.spartabots.frc2015.Robot;
import org.spartabots.frc2015.action.Actions;

public class AutoProfile1 extends Profile {

	public AutoProfile1(Robot robot) {
		super(Profile.AUTONOMOUS, robot, true);
	}
	
	@Override
	public void init() {
		robot.drive.resetGyro();
		robot.drive.resetEncoders();
		
		robot.elevator.clampE1In();
		
		Actions.rotateDeg(90, 0.6); 	// Rotate robot to point to center of field
		Actions.driveTime(6000, 0.85); 	// Drive into auto zone
	}

	@Override
	public void autoPeriodic() {
	}

	
	@Override
	public void done() {
		
	}

}
