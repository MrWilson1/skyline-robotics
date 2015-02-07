package org.spartabots.frc2015.profile;

import org.spartabots.frc2015.Robot;
import org.spartabots.frc2015.action.Actions;

public class AutoProfile4 extends Profile {

	public AutoProfile4(Robot robot) {
		super(Profile.AUTONOMOUS, robot);
	}
	
	@Override
	public void init() {https://skyline-robotics.googlecode.com/svn
		robot.drive.resetGyro();
		robot.drive.resetEncoders();
		
		robot.elevator.clampE1In();
		robot.elevator.setE1(1);
		Actions.waitAction(100);
		robot.elevator.setE1(0);
		Actions.rotateDeg(-90, 0.6); 	// Rotate robot to point to center of field
		Actions.driveTime(6000, 0.85); 	// Drive into auto zone
	
		
	}

	@Override
	public void autoPeriodic() {
		
	}

	
	@Override
	public void done() {
		
	}

}
