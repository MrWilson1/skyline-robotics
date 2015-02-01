package org.spartabots.frc2015.profile;

import org.spartabots.frc2015.Robot;
import org.spartabots.frc2015.action.Actions;

public class AutoProfile3 extends Profile {

	public AutoProfile3(Robot robot) {
		super(Profile.AUTONOMOUS, robot);
	}
	
	@Override
	public void init() {
		robot.drive.resetGyro();
		robot.drive.resetEncoders();
		
		robot.elevator.clampE1In();
		robot.elevator.setE1(1);
		robot.drive.traverse.set(-1);
		Actions.driveTime(300, 0.85);
		robot.drive.traverse.set(3);
		robot.elevator.clampE1Out();
		robot.elevator.setE1(-1);
		
		robot.elevator.clampE1In();
		robot.elevator.setE1(1);
		robot.drive.traverse.set(-1);
		Actions.driveTime(300, 0.85);
		robot.drive.traverse.set(1);
		robot.elevator.clampE1Out();
		robot.elevator.setE1(-1);
		
		robot.elevator.clampE1In();
		robot.elevator.setE1(1);
		robot.drive.traverse.set(-1);
		Actions.driveTime(300, 0.85);
		robot.drive.traverse.set(1);
		robot.elevator.clampE1Out();
		robot.elevator.setE1(-1);
		
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
