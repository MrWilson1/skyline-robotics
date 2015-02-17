package org.spartabots.frc2015.auto;

import org.spartabots.frc2015.action.Actions;
import org.spartabots.frc2015.action.LogAction;
import org.spartabots.frc2015.action.SeriesAction;
import org.spartabots.frc2015.auto.AutoHandle;

public class AutoTest extends SeriesAction {
	int i = 1;
	
	@Override
	public void init() {
		enqueue(new LogAction("AutoTest", "run count=" + i));
		enqueueToProfile(Actions.elevatorGoto(AutoHandle.TOTE_HEIGHT));
		enqueue(Actions.rotateDeg(90, 0.6));
		enqueue(new LogAction("AutoTest Queue", this.queue.toString()));
	}
	
}
