package org.spartabots.frc2015.auto;

import org.spartabots.frc2015.action.Actions;
import org.spartabots.frc2015.action.LogAction;
import org.spartabots.frc2015.action.SeriesAction;

public interface AutoHandle {
	public static final double TOTE_TRAVERSE_DIST = 1.64; // neg to go left, pos to go right (1.62) ?
	public static final double TOTE_TRAVERSE_DIST_BACK = 1.68;
	public static final double TOTE_HEIGHT = 1.42 + 0.8;
	public static final double FLOOR_HEIGHT = 0;
	public static final double TOTE_LENGTH = 2.25;
	public static final double BIN_LENGTH = 1.84;
	public static final double TOTE_BIN_LENGTH = TOTE_LENGTH+BIN_LENGTH - 0.925; // ???
	public static final double ELEVATOR_AFTER_PICKUP_WAIT_TIME = 1500;
	public static final double AFTER_DRIVE_WAIT_TIME = 1000;
	public static final double DRIVE_BACK_TO_PICKUP_DIST = 0.5;
	
	public static void enqueueRepickup(SeriesAction a) {
		a.enqueue(Actions.clampOut()); // drop tote over other tote
		a.enqueue(Actions.driveDist(-DRIVE_BACK_TO_PICKUP_DIST, 0.6));
		a.enqueue(Actions.elevatorGoto(FLOOR_HEIGHT)); // bring elevatorTime down to bottom most tote
		a.enqueue(Actions.driveDist(DRIVE_BACK_TO_PICKUP_DIST, 0.6));
		a.enqueue(Actions.clampIn()); // pick up both totes (which should now be stacked)
	}
	
	public static void navigate(SeriesAction a) {
		a.enqueueToProfile(Actions.elevatorGoto(TOTE_HEIGHT*3));
		a.enqueue(Actions.waitAction(ELEVATOR_AFTER_PICKUP_WAIT_TIME));
		a.enqueue(Actions.traverseDist(TOTE_TRAVERSE_DIST, 0.8));
		a.enqueue(Actions.driveDist(TOTE_BIN_LENGTH, 0.85));
		a.enqueue(Actions.waitAction(AFTER_DRIVE_WAIT_TIME));
		a.enqueue(Actions.traverseDist(-TOTE_TRAVERSE_DIST_BACK, 0.8));
	}
	
	public static void enqueue1stTote(SeriesAction a, boolean expectingNext) {
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
		enqueueRepickup(a); // pick up
		if (expectingNext) {
			navigate(a);
		} else {
			a.enqueueToProfile(Actions.elevatorGoto(TOTE_HEIGHT/2));
		}
	}
	
	public static void enqueue3rdTote(SeriesAction a, boolean expectingNext) {
		a.enqueue(new LogAction("Auto Step", "3rd Tote, " + expectingNext));
		a.enqueue(Actions.elevatorGoto(TOTE_HEIGHT)); // bring elevatorTime down on top of tote
		enqueueRepickup(a); // pick up
	}
	
	public static void enqueueGotoAutoZone(SeriesAction a, boolean turnOpposite) {
		a.enqueue(new LogAction("Auto Step", "Auto zone, " + turnOpposite));
		a.enqueue(Actions.rotateDeg((turnOpposite) ? -90 : 90, 0.5)); // Rotate robot to face auto zone
		a.enqueueToProfile(Actions.elevatorGoto(FLOOR_HEIGHT)); // bring elevator down to right above floor
		a.enqueue(Actions.driveDist(6, 1)); // Drive to auto zone
		a.enqueue(Actions.clampOut()); // Let go of all totes
		a.enqueue(Actions.driveDist(-2, 0.8)); // Back off
	}
}
