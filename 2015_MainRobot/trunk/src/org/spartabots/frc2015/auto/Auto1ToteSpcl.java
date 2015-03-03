package org.spartabots.frc2015.auto;

import org.spartabots.frc2015.action.Actions;
import org.spartabots.frc2015.action.ResetAction;
import org.spartabots.frc2015.action.SeriesAction;

public class Auto1ToteSpcl extends SeriesAction {
	
	@Override
	public void init() {
		this.enqueue(new ResetAction(ResetAction.GYRO));
		this.enqueue(new ResetAction(ResetAction.ZERO_GYRO));
		this.enqueue(Actions.clampIn());
		this.enqueue(Actions.driveDist(2, 0.57, false));
		this.enqueue(Actions.rotateDeg(120 * (int) robot.autoRotateChooser.getSelected(), 0.625, true));
	}
	
}
