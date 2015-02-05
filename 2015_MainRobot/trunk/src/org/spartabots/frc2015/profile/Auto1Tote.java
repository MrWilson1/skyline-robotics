package org.spartabots.frc2015.profile;

import org.spartabots.frc2015.Robot;
import org.spartabots.frc2015.action.Actions;

public class Auto1Tote extends Profile {

	public Auto1Tote(Robot robot) {
		super(Profile.AUTONOMOUS, robot, true);
	}
	
	@Override
	public void init() {
		robot.drive.resetGyro();
		robot.drive.resetEncoders();
		
		robot.elevator.clampIn();
		
		robot.elevator.setElevator(0.85);		//elevator up
		Actions.waitAction(25);
		
		Actions.rotateDeg(90, 0.6); 	// Rotate robot to point to center of field
		Actions.driveTime(6000, 0.85); 	// Drive into auto zone
		
		robot.elevator.setElevator(-0.45);	//elevator down
		Actions.waitAction(45);
		
		robot.elevator.clampOut();
	}

	@Override
	public void autoPeriodic() {
	}

	
	@Override
	public void done() {
		
	}

}
