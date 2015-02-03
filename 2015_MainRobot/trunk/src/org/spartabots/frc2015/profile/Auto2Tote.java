package org.spartabots.frc2015.profile;

import org.spartabots.frc2015.Robot;
import org.spartabots.frc2015.action.Actions;

public class Auto2Tote extends Profile {

	public Auto2Tote(Robot robot) {
		super(Profile.AUTONOMOUS, robot);
	}
	
	@Override
	public void init() {
		robot.drive.resetGyro();
		robot.drive.resetEncoders();
		
		robot.elevator.clampE1In();
		robot.elevator.clampE2In();
		Actions.waitAction(25);

		/*traverse robot 2 feet 
		forward to position
		traverse robot - 2 feet 
		elevator 1 up 
		forward a little bit 
		grip 2 
		turn 
		forward
		*/
		
	}

	@Override
	public void autoPeriodic() {
		
	}

	
	@Override
	public void done() {
		
	}

}
