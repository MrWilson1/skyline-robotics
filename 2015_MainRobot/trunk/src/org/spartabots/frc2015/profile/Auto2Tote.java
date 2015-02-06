package org.spartabots.frc2015.profile;

import org.spartabots.frc2015.Robot;
import org.spartabots.frc2015.action.Actions;
import org.spartabots.frc2015.action.ParallelAction;
import org.spartabots.frc2015.action.SeriesAction;
import org.spartabots.frc2015.util.Util;

public class Auto2Tote extends Profile {

	public Auto2Tote(Robot robot) {
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
