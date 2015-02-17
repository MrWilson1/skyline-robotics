package org.spartabots.frc2015.auto;

import org.spartabots.frc2015.action.SeriesAction;

public class Auto3Tote extends SeriesAction {
	
	@Override
	public void init() {
		AutoHandle.enqueue1stTote(this, true);
		AutoHandle.enqueue2ndTote(this, true);
		AutoHandle.enqueue3rdTote(this, false);
		AutoHandle.enqueueGotoAutoZone(this, false);
	}
	
}
