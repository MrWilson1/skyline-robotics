package org.spartabots.frc2015.profile;

import org.spartabots.frc2015.Robot;
import org.spartabots.frc2015.action.Actions;

public class Auto1Bin extends Profile {

	public Auto1Bin(Robot robot) {
		super(Profile.AUTONOMOUS, robot);
	}
	
	@Override
	public void init() {
		robot.drive.resetGyro();
		robot.drive.resetEncoders();
		
		robot.elevator.clampIn();
		robot.elevator.setElevator(1);
		Actions.waitAction(100);
		robot.elevator.setElevator(0);
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
