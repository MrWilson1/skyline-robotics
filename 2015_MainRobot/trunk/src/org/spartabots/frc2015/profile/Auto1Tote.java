package org.spartabots.frc2015.profile;

import org.spartabots.frc2015.Robot;
import org.spartabots.frc2015.action.Actions;
import org.spartabots.frc2015.action.ParallelAction;
import org.spartabots.frc2015.action.SeriesAction;
import org.spartabots.frc2015.util.Util;

public class Auto1Tote extends Profile {

	public Auto1Tote(Robot robot) {
		super(Profile.AUTONOMOUS, robot, true);
	}
	
	@Override
	public void init() {
		robot.drive.resetGyro();
		robot.drive.resetEncoders();
		
		SeriesAction series = new SeriesAction();
		series.add(Actions.clampIn());
		series.add(new ParallelAction(Actions.elevatorTime(0.85, 2000)));

		// Drive to auto zone
		series.add(Actions.rotateDeg(90, 0.6)); // Rotate robot to face auto zone
		series.add(new ParallelAction(Actions.elevatorTime(-0.85, 2500))); // bring elevator down to right above floor
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
