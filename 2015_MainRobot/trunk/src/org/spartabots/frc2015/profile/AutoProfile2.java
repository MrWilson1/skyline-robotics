package org.spartabots.frc2015.profile;

import org.spartabots.frc2015.Robot;

public class AutoProfile2 extends Profile {

	public AutoProfile2(Robot robot) {
		super(Profile.AUTONOMOUS, robot);
	}
	
	@Override
	public void init() {
		robot.drive.resetGyro();
		robot.drive.resetEncoders();
		robot.elevator.clampE1In();
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
