package org.spartabots.frc2015.profile;

import org.spartabots.frc2015.Robot;
import org.spartabots.frc2015.action.Actions;
import org.spartabots.frc2015.util.Util;

// TODO ESTIMATED VALUES ARE CURRENTLY BEING USED, change values to actual
// TODO move these actions into series and parallel and put them into functions that all 4 auto modes can use
public class Auto3Tote extends Profile {

	public Auto3Tote(Robot robot) {
		super(Profile.AUTONOMOUS, robot, true);
	}
	
	@Override
	public void init() {
		robot.drive.resetGyro();
		robot.drive.resetEncoders();
		
		// Tote 1
		Actions.clampIn(); // grab first tote
		Actions.elevator(0.85, 2000); // bring elevator up so that tote can be dropped over another
		Actions.traverseTime(200, 0.5);
		Actions.driveDist(Util.toMeters(2.75), 0.85);
		Actions.traverseTime(200, -0.5);
		
		// Tote 2
		Actions.elevator(-0.75, 500); // bring elevator down a bit
		Actions.clampOut(); // drop tote over other tote
		Actions.elevator(-0.85, 2000); // bring elevator down to bottom most tote
		Actions.clampIn(); // pick up both totes (which should now be stacked)
		Actions.traverseTime(200, 0.5);
		Actions.driveDist(Util.toMeters(1.5), 0.85);
		Actions.traverseTime(200, -0.5);
		
		Actions.elevator(0.85, 2500); // bring elevator back up
		
		// Tote 3
		Actions.elevator(-0.75, 500); // bring elevator down a bit
		Actions.clampOut(); // drop tote over other tote
		Actions.elevator(-0.85, 2000); // bring elevator down to bottom most tote
		Actions.clampIn(); // pick up all three totes
		
		// Drive to auto zone
		Actions.rotateDeg(-90, 0.6); // Rotate robot to face auto zone
		Actions.driveDist(Util.toMeters(8.5) ,0.85); // Drive to auto zone
		Actions.clampOut(); // Let go of all totes
		Actions.driveDist(Util.toMeters(-2), -0.85);
	}

	@Override
	public void autoPeriodic() {
		
	}

	
	@Override
	public void done() {
		
	}

}
