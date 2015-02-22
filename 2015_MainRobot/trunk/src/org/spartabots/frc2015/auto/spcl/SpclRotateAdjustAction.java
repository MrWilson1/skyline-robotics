package org.spartabots.frc2015.auto.spcl;

import org.spartabots.frc2015.action.Action;

public class SpclRotateAdjustAction extends Action {
	
	public SpclRotateAdjustAction() {
		this.setTimeout(200);
	}

	@Override
	public void init() {
		
	}

	@Override
	public boolean runPeriodic() {
		robot.drive.m_drive.arcadeDrive(0, -(robot.drive.getGyroAngle() * 0.032));
		return true;
	}

	@Override
	public void done() {
		
	}
	
}