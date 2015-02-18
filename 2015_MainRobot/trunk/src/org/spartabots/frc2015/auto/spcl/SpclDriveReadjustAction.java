package org.spartabots.frc2015.auto.spcl;

import org.spartabots.frc2015.action.DriveAction;
import org.spartabots.frc2015.util.Util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SpclDriveReadjustAction extends DriveAction {
	double adjustTo;
	double my_speed = 0;
	
	boolean zeroEcAfter = true;
	public SpclDriveReadjustAction(double adjustTo, boolean zeroEcAfter, double speed) {
		super(DISTANCE, 0.1, 0.5, false); /* dummy values, second param cannot be 0 */
		this.adjustTo = adjustTo;
		SmartDashboard.putString("Spcl Drive Did Init", "true; " + this.init + ", " + this.doneFinal);
		this.zeroEcAfter = zeroEcAfter;
		this.my_speed = speed;
	}
	
	@Override
	public void init() {
		SmartDashboard.putBoolean("Spcl Drive Did run", true);
		this._init(getDist(adjustTo, zeroEcAfter), this.my_speed);
	}
	
	public static double getDist(double adjustTo, boolean zeroEcAfter) {
		double dist = 0;
		String s = null;
		if (robot.drive.getBeltEncoderDistance() > adjustTo) { // overshot
			dist = -(robot.drive.getBeltEncoderDistance() - adjustTo);
			s = "overshot";
		}
		if (robot.drive.getBeltEncoderDistance() < adjustTo) { // too short
			dist = (adjustTo - robot.drive.getBeltEncoderDistance());
			s = "too short";
		}
		SmartDashboard.putString("Spcl Drive", Util.round2d(robot.drive.getBeltEncoderDistance()) +
				", adjust_dist=" + Util.round2d(dist) + ", adjust_to=" + adjustTo + ", state=" + s);
		if (zeroEcAfter)
			robot.drive.setZeroEcDist();
		return dist;
	}
}