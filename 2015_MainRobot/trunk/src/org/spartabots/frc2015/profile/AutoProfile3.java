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
		
		//tote1
		robot.elevator.clampE1In();		
		robot.elevator.setE1(0.85);		//elevator up
		traversedrive(200, 0.5);
		Actions.driveTime(1000, 0.85);
		traversedrive(200, -0.5);
		
		//tote2
		robot.elevator.clampE2In();
		robot.elevator.setE2(0.85);		//elevator up
		traversedrive(200, 0.5);
		Actions.driveTime(300, 0.85);
		traversedrive(200, -0.5);
		
		//tote3
		Actions.driveTime(500,0.5);		//Drive to third tote
		Actions.rotateDeg(-90, 0.6); 	// Rotate robot 
		
		Actions.driveTime(2000, 0.85); 	// Drive into auto zone
		robot.elevator.clampE1Out();	//Let go of totes
		robot.elevator.clampE2Out();
		Actions.driveTime(1000, -0.25); //Back up
	}
	
	public static void traversedrive (int time, double power)
	{
		robot.drive.traverse.set(power);
		Actions.waitAction(time);
		robot.drive.traverse.set(0); 
	}

	@Override
	public void autoPeriodic() {
		
	}

	
	@Override
	public void done() {
		
	}

}
