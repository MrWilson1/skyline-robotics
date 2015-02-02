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
		encoderdrive (2.75, 0.85);
		traversedrive(200, -0.5);
		
		//tote2
		robot.elevator.clampE2In();
		robot.elevator.setE2(0.85);		//elevator up
		traversedrive(200, 0.5);
		encoderdrive (1.5, 0.85);
		traversedrive(200, -0.5);
		
		//tote3
		encoderdrive (1.25,0.85);
		Actions.rotateDeg(-90, 0.6); 	// Rotate robot to face auto zone
		encoderdrive (8.5,0.85);		//Drive to auto zone
		robot.elevator.clampE1Out();	//Let go of totes
		robot.elevator.clampE2Out();
		encoderdrive (-2,-0.85);
	}
	
	public  void traversedrive (int time, double power)
	{
		robot.drive.traverse.set(power);
		Actions.waitAction(time);
		robot.drive.traverse.set(0); 
	}
	
	public  void encoderdrive (double distance, double power) 
	{
		robot.drive.resetEncoders(); 
		
		while (robot.drive.getLeftEncoderDistance()>=distance)
		{
			Actions.driveTime(1, power); //method for driving without time?
		}
	}

	@Override
	public void autoPeriodic() {
		
	}

	
	@Override
	public void done() {
		
	}

}
