package org.spartabots.frc2015.profile;

import org.spartabots.frc2015.Robot;
import org.spartabots.frc2015.action.Actions;
import org.spartabots.frc2015.util.Constants;

public class AutoProfile1 extends Profile {

	public AutoProfile1(Robot robot) {
		super(Profile.AUTONOMOUS, robot);
	}
	
	@Override
	public void init() {
		robot.drive.resetGyro();
		robot.drive.resetEncoders();
		Actions.rotateDeg(90);
	}

	@Override
	public void autoPeriodic() {
	}

	
	@Override
	public void done() {
		
	}

}
