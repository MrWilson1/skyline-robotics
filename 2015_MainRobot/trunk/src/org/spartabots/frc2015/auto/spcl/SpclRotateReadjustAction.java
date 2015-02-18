package org.spartabots.frc2015.auto.spcl;

import org.spartabots.frc2015.action.Action;

public class SpclRotateReadjustAction extends Action {
	
	public SpclRotateReadjustAction() {
		this.setTimeout(600);
	}

	@Override
	public void init() {
		
	}

	@Override
	public boolean runPeriodic() {
		robot.drive.m_drive.arcadeDrive(0, -(robot.drive.getGyroAngle() * 0.03));
		return true;
	}

	@Override
	public void done() {
		
	}
	
}