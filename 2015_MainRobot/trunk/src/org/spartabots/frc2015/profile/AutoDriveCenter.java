package org.spartabots.frc2015.profile;

import org.spartabots.frc2015.Robot;
import org.spartabots.frc2015.action.Actions;
import org.spartabots.frc2015.util.Util;

public class AutoDriveCenter extends Profile {

	public AutoDriveCenter(Robot robot) {
		super(Profile.AUTONOMOUS, robot, true);
	}

	
	@Override
	public void init() {
		Actions.driveDist(Util.toMeters(9), 0.85).run();
	}

	@Override
	public void autoPeriodic() {
		
	}

	
	@Override
	public void done() {
		
	}
}
