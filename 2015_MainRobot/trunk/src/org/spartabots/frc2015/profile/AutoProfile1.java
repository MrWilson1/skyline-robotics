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
		robot.elevator.clampE2In();
		
		robot.elevator.setE1(0.85);		//elevator up
		robot.elevator.setE2(0.85);
		Actions.waitAction(25);
		
		Actions.rotateDeg(90, 0.6); 	// Rotate robot to point to center of field
		Actions.driveTime(6000, 0.85); 	// Drive into auto zone
		
		robot.elevator.setE1(-0.45);	//elevator down
		robot.elevator.setE2(-0.45);
		Actions.waitAction(45);
		
		robot.elevator.clampE1Out();
		robot.elevator.clampE2Out();
	}

	@Override
	public void autoPeriodic() {
	}

	
	@Override
	public void done() {
		
	}

}
