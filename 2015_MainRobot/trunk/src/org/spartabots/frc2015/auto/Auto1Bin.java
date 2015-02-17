package org.spartabots.frc2015.auto;

import org.spartabots.frc2015.action.SeriesAction;

public class Auto1Bin extends SeriesAction {
	
	@Override
	public void init() {
		AutoHandle.enqueue1stTote(this, true); /* works with bin too */
		AutoHandle.enqueueGotoAutoZone(this, true);
	}
	
}
