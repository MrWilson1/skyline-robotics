package org.spartabots.frc2015.auto.spcl;

import org.spartabots.frc2015.action.TraverseAction;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SpclTraverseBackAction extends TraverseAction {
	public SpclTraverseBackAction() {
		super(DISTANCE, 0.1, 0.5, false); /* dummy values, second param cannot be 0 */
		SmartDashboard.putString("Spcl Traverse Did Init", "true; " + this.init + ", " + this.doneFinal);
	}
	
	@Override
	public void init() {
		this._init(getDist(), 1);
	}
	
	public static double getDist() {
		SmartDashboard.putNumber("Spcl Traverse Dist", -robot.drive.getTraverseEncoderDistance());
		
		double val = -robot.drive.getTraverseEncoderDistance();
		int val_zsgn = (int) Math.signum(val);
		
		val = (Math.abs(val) + 0.3) * val_zsgn;
		
		return val;
	}
}