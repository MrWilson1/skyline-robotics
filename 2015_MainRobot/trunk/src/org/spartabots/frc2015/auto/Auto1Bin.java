package org.spartabots.frc2015.auto;

import org.spartabots.frc2015.action.Actions;
import org.spartabots.frc2015.action.SeriesAction;
import org.spartabots.frc2015.util.Util;

public class Auto1Bin extends SeriesAction {
	
	@Override
	public void init() {
		robot.drive.resetGyro();
		robot.drive.resetEncoders();
		
		enqueue(Actions.clampIn());
		enqueueToProfile(Actions.elevatorTime(0.85, 2000));

		// Drive to auto zone
		enqueue(Actions.rotateDeg(-90, 0.6)); // Rotate robot to face auto zone
		enqueueToProfile(Actions.elevatorTime(-0.85, 2500)); // bring elevator down to right above floor
		enqueue(Actions.driveDist(Util.toMeters(8.5) ,0.85)); // Drive to auto zone
		enqueue(Actions.clampOut()); // Let go of all totes
		enqueue(Actions.driveDist(Util.toMeters(-2), 0.85)); // Back off
	}

}
