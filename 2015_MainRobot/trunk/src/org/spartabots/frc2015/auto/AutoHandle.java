package org.spartabots.frc2015.auto;

import org.spartabots.frc2015.action.Actions;
import org.spartabots.frc2015.action.LogAction;
import org.spartabots.frc2015.action.ResetAction;
import org.spartabots.frc2015.action.SeriesAction;
import org.spartabots.frc2015.auto.spcl.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public interface AutoHandle {
	public static final double TOTE_TRAVERSE_DIST = -2.8; // -1.64 for right (pos), ??? for left (neg)
	public static final double TOTE_HEIGHT = 1.22;
	public static final double FLOOR_HEIGHT = 0;
	public static final double TOTE_LENGTH = 2.25;
	public static final double BIN_LENGTH = 1.84;
	public static final double INITIAL_DRIVE_DIST = 3.76; // ??? (3.165... ???)
	public static final double REAL_DRIVE_DIST = 6.32; // 6.41 actually-ish
	public static final double ELEVATOR_AFTER_PICKUP_WAIT_TIME = 1500;
	public static final double AFTER_DRIVE_WAIT_TIME = 1000;
	public static final double DRIVE_BACK_TO_PICKUP_DIST = 0.5;
	
	public static void repickup(SeriesAction a) {
		a.enqueue(Actions.clampOut()); // drop tote over other tote
		a.enqueue(Actions.driveDist(-DRIVE_BACK_TO_PICKUP_DIST, 0.6, true));
		a.enqueue(Actions.elevatorGoto(FLOOR_HEIGHT)); // bring elevatorTime down to bottom most tote
		a.enqueue(Actions.driveDist(DRIVE_BACK_TO_PICKUP_DIST, 0.6, true));
		a.enqueue(Actions.clampIn()); // pick up both totes (which should now be stacked)
	}
	
	public static void navigate(SeriesAction a) {
		a.enqueueToProfile(Actions.elevatorGoto(TOTE_HEIGHT*3));
		a.enqueue(Actions.waitAction(ELEVATOR_AFTER_PICKUP_WAIT_TIME));
		a.enqueue(Actions.traverseDist(TOTE_TRAVERSE_DIST, 1, false));
		a.enqueue(Actions.driveDist(INITIAL_DRIVE_DIST, 0.8, false));
		a.enqueue(new LogAction(null, null) {
			public void init() {
				SmartDashboard.putNumber("Norm Drive", robot.drive.getBeltEncoderDistance());
				SmartDashboard.putString("Spcl Drive Did Init", "false");
				SmartDashboard.putBoolean("Spcl Drive Did run", false);
				SmartDashboard.putNumber("Traverse Dist", robot.drive.getTraverseEncoderDistance());
				cancel();}});
		a.enqueue(new SpclDriveReadjustAction(REAL_DRIVE_DIST-0.11, false, 0.52)); // first try
		a.enqueue(new SpclDriveReadjustAction(REAL_DRIVE_DIST, true, 0.4975)); // second try
		a.enqueue(new LogAction(null, null) {
			public void init() {
				SmartDashboard.putNumber("Spcl Drive After", robot.drive.getBeltEncoderDistance());
				cancel();}});
		a.enqueue(new SpclRotateReadjustAction());
		a.enqueue(new LogAction(null, null) {
			public void init() {
				SmartDashboard.putNumber("Spcl Rotate After", robot.drive.getBeltEncoderDistance());
				SmartDashboard.putString("Spcl Traverse Did Init", "false");
				cancel();}});
		a.enqueue(Actions.waitAction(AFTER_DRIVE_WAIT_TIME));
		a.enqueue(new SpclTraverseBackAction());
		//a.enqueue(Actions.traverseDist(-TOTE_TRAVERSE_DIST_BACK, 1));
	}
	
	public static void enqueue1stTote(SeriesAction a, boolean expectingNext) {
		a.enqueue(new ResetAction(ResetAction.GYRO));
		a.enqueue(new ResetAction(ResetAction.ZERO_GYRO)); // reset properly you scrub
		a.enqueue(new LogAction("Auto Step", "1st Tote, " + expectingNext));
		a.enqueue(Actions.clampIn());
		if (expectingNext) {
			navigate(a);
		} else {
			a.enqueueToProfile(Actions.elevatorGoto(TOTE_HEIGHT/2));
		}
	}
	
	public static void enqueue2ndTote(SeriesAction a, boolean expectingNext) {
		a.enqueue(new LogAction("Auto Step", "2nd Tote, " + expectingNext));
		a.enqueue(Actions.elevatorGoto(TOTE_HEIGHT)); // bring elevatorTime down on top of tote
		repickup(a); // pick up
		if (expectingNext) {
			navigate(a);
		} else {
			a.enqueueToProfile(Actions.elevatorGoto(TOTE_HEIGHT/2));
		}
	}
	
	public static void enqueue3rdTote(SeriesAction a, boolean expectingNext) {
		a.enqueue(new LogAction("Auto Step", "3rd Tote, " + expectingNext));
		a.enqueue(Actions.elevatorGoto(TOTE_HEIGHT)); // bring elevatorTime down on top of tote
		repickup(a); // pick up
	}
	
	public static void enqueueGotoAutoZone(SeriesAction a, boolean turnOpposite) {
		a.enqueue(new LogAction("Auto Step", "Auto zone, " + turnOpposite));
		a.enqueue(Actions.rotateDeg((turnOpposite) ? -90 : 90, 0.6)); // Rotate robot to face auto zone
		a.enqueueToProfile(Actions.elevatorGoto(FLOOR_HEIGHT)); // bring elevator down to right above floor
		a.enqueue(Actions.driveDist(6, 1, true)); // Drive to auto zone
		a.enqueue(Actions.clampOut()); // Let go of all totes
		a.enqueue(Actions.driveDist(-2, 0.8, true)); // Back off
	}
}
