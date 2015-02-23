package org.spartabots.frc2015.action;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LogAction extends Action {
	String key;
	String value;
	
	public LogAction(String key, String value) {
		this.key = key;
		this.value = value;
		this.canTimeOut = false;
	}
	
	@Override
	public void init() {
		SmartDashboard.putString(key, value);
		cancel();
	}

	@Override
	public boolean runPeriodic() {
		return false;
	}

	@Override
	public void done() {
	}
	
}