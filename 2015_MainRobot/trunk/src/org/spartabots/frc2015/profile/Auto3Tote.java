package org.spartabots.frc2015.profile;

import org.spartabots.frc2015.Robot;
import org.spartabots.frc2015.action.*;
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
		SeriesAction series = new SeriesAction();
		series.add(Actions.clampIn());
		series.add(new ParallelAction(Actions.elevatorTime(0.85, 2000)));
		series.add(Actions.traverseTime(200, 0.5));
		series.add(Actions.driveDist(Util.toMeters(2.75), 0.85));
		series.add(Actions.traverseTime(-200, 0.5));
		
		// Tote 2
		series.add(Actions.elevatorTime(-0.75, 500)); // bring elevatorTime down a bit
		series.add(Actions.clampOut()); // drop tote over other tote
		series.add(Actions.elevatorTime(-0.85, 2000)); // bring elevatorTime down to bottom most tote
		series.add(Actions.clampIn()); // pick up both totes (which should now be stacked)
		series.add(new ParallelAction(Actions.elevatorTime(0.85, 2500))); // bring elevatorTime back up
		series.add(Actions.traverseTime(200, 0.5));
		series.add(Actions.driveDist(Util.toMeters(1.5), 0.85));
		series.add(Actions.traverseTime(-200, 0.5));
		
		// Tote 3
		series.add(Actions.elevatorTime(-0.75, 500)); // bring elevatorTime down a bit
		series.add(Actions.clampOut()); // drop tote over other tote
		series.add(Actions.elevatorTime(-0.85, 2000)); // bring elevatorTime down to bottom most tote
		series.add(Actions.clampIn()); // pick up all three totes
		series.add(new ParallelAction(Actions.elevatorTime(0.85, 2500))); // bring elevatorTime back up
		
		// Drive to auto zone
		series.add(Actions.rotateDeg(90, 0.6)); // Rotate robot to face auto zone
		series.add(new ParallelAction(Actions.elevatorTime(-0.85, 2500))); // bring elevatorTime down to right above floor
		series.add(Actions.driveDist(Util.toMeters(8.5) ,0.85)); // Drive to auto zone
		series.add(Actions.clampOut()); // Let go of all totes
		series.add(Actions.driveDist(Util.toMeters(-2), 0.85)); // Back off
		
		series.run();
	}

	@Override
	public void autoPeriodic() {
		
	}

	
	@Override
	public void done() {
		
	}

}
