package org.spartabots.frc2015.auto;

import org.spartabots.frc2015.action.*;
import org.spartabots.frc2015.auto.spcl.*;

// TOTES MAH GOATS
public interface AutoHandle {
	public static final double TOTE_TRAVERSE_DIST = -2.8; // -1.64 for right (pos), ??? for left (neg)
	public static final double TOTE_HEIGHT = 1.22;
	public static final double FLOOR_HEIGHT = 0;
	public static final double TOTE_LENGTH = 2.25;
	public static final double BIN_LENGTH = 1.84;
	public static final double INITIAL_DRIVE_DIST = 3.8; // ??? (3.165... ???)
	public static final double NAV2_INTIAL_DRIVE_DIST = 4.6; // 6.41 actually-ish
	public static final double REAL_DRIVE_DIST = 6.23; // 6.41 actually-ish
	public static final double ELEVATOR_AFTER_PICKUP_WAIT_TIME_NAV1 = 1500;
	public static final double ELEVATOR_AFTER_PICKUP_WAIT_TIME_NAV2 = 800;
	public static final double ELEVATOR_AFTER_PICKUP_WAIT_TIME_NAV2_PART2 = 250;
	public static final double AFTER_DRIVE_WAIT_TIME = 1000;
	public static final double DRIVE_BACK_TO_PICKUP_DIST = 1.25; // 1.5
	public static final double DRIVE_BACK_TO_PICKUP_DIST_EXTRA2 = 0.15; // 1.25
	
	public static final double ROTATE_VAL = -20;
	public static final double ROTATE_VAL2 = 40;
	public static final double STACK_EXTRA_HEIGHT = 0.2;
	public static final double NAV2_TRAVERSE_BACK = 2;
	
	public static void repickup(SeriesAction a) {
		a.enqueue(Actions.clampOut()); // drop tote over other tote
		a.enqueue(Actions.driveDist(-DRIVE_BACK_TO_PICKUP_DIST, 0.74, true));
		a.enqueue(Actions.elevatorGoto(FLOOR_HEIGHT)); // bring elevator down to bottommost tote
		a.enqueue(new ResetAction(ResetAction.TRAVERSE_EC));
		a.enqueue(Actions.driveDist(DRIVE_BACK_TO_PICKUP_DIST + DRIVE_BACK_TO_PICKUP_DIST_EXTRA2, 0.74, true));
		a.enqueue(Actions.clampIn()); // pick up stack totes
		a.enqueue(Actions.waitAction(150));
	}

	public static void navigate(SeriesAction a) {
		a.enqueue(Actions.rotateDeg(ROTATE_VAL, 0.7, true));
		a.enqueue(Actions.waitAction(30));
		a.enqueue(new ResetAction(ResetAction.ZERO_EC));
		a.enqueue(Actions.driveDist(1.5, 0.7825, false));
		a.enqueue(Actions.waitAction(150));
		a.enqueue(Actions.rotateDeg(ROTATE_VAL2, 1, true));
		a.enqueue(new ResetAction(ResetAction.RESET_GYRO_OFFSET));
		a.enqueue(new RotateToZero(0.65));
		a.enqueue(new SpclRotateAdjustAction());
		a.enqueue(new TraverseAction(TraverseAction.DISTANCE, NAV2_TRAVERSE_BACK, 0.985, false));
		a.enqueue(new RotateToZero(0.65));
		a.enqueue(new SpclRotateAdjustAction());
		a.enqueueToProfile(Actions.elevatorGoto(TOTE_HEIGHT*2+STACK_EXTRA_HEIGHT));
		a.enqueue(Actions.waitAction(ELEVATOR_AFTER_PICKUP_WAIT_TIME_NAV2_PART2));
		a.enqueue(new SpclDriveAdjustAction(NAV2_INTIAL_DRIVE_DIST, false, 0.75));
		a.enqueue(new SpclDriveAdjustAction(REAL_DRIVE_DIST, true, 0.52));
	}
	
	public static void enqueue1stTote(SeriesAction a, boolean expectingNext) {
		a.enqueue(new ResetAction(ResetAction.GYRO));
		a.enqueue(new ResetAction(ResetAction.ZERO_GYRO));
		a.enqueue(new LogAction("Auto Step", "1st Tote, " + expectingNext));
		a.enqueue(Actions.clampIn());
		if (expectingNext) {
			navigate(a);
		} else {
			a.enqueue(Actions.elevatorGoto(TOTE_HEIGHT/2));
		}
	}
	
	public static void enqueue2ndTote(SeriesAction a, boolean expectingNext) {
		a.enqueue(new LogAction("Auto Step", "2nd Tote, " + expectingNext));
		a.enqueue(Actions.elevatorGoto(TOTE_HEIGHT+STACK_EXTRA_HEIGHT)); // bring elevatorTime down on top of tote
		repickup(a); // pick up
		if (expectingNext) {
			navigate(a);
		} else {
			a.enqueue(Actions.elevatorGoto(TOTE_HEIGHT));
		}
	}
	
	public static void enqueue3rdTote(SeriesAction a, boolean expectingNext) {
		a.enqueue(new LogAction("Auto Step", "3rd Tote, " + expectingNext));
		a.enqueue(Actions.elevatorGoto(TOTE_HEIGHT+STACK_EXTRA_HEIGHT)); // bring elevatorTime down on top of tote
		repickup(a); // pick up
	}
	
	public static void enqueueGotoAutoZone(SeriesAction a, boolean turnOpposite) {
		a.enqueue(new LogAction("Auto Step", "Auto zone, " + turnOpposite));
		a.enqueue(Actions.rotateDeg((turnOpposite) ? -90 : 90, 0.575, true)); // Rotate robot to face auto zone
		a.enqueueToProfile(Actions.elevatorGoto(FLOOR_HEIGHT)); // bring elevator down to right above floor
		a.enqueue(Actions.driveDist(8, 0.7, true)); // Drive to auto zone
		a.enqueue(Actions.rotateDeg(-45, 0.625, true)); // Rotate robot to face auto zone
	}
}