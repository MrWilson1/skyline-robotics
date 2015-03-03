package org.spartabots.frc2015.auto;

import org.spartabots.frc2015.action.SeriesAction;

public class Auto1Tote extends SeriesAction {
	
	@Override
	public void init() {
		AutoHandle.enqueue1stTote(this, false);
		AutoHandle.enqueueGotoAutoZone(this);
	}
	
}
