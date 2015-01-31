package org.spartabots.frc2015.action;

import java.util.concurrent.CyclicBarrier;

public class Actions {

	
	/**
	 * Requests multiple actions to be run concurrently in its own thread starting at the same time.
	 * If only one action is given, the action will be run in its own thread.
	 * 
	 * @param actions
	 * @return Whether request was accepted. May return false if the thread pool has no available threads
	 * to process the request.
	 */
	public static boolean runConcurrent(Action... actions) {
		if (actions.length == 0)
			return false;
		if (actions.length == 1) // Only one, no need for gate & for lloop
			return (ActionThread.request(actions[0], null) != null);
		
		final CyclicBarrier gate = new CyclicBarrier(actions.length);
		for (final Action a : actions) {
			if (ActionThread.request(a, gate) == null) {
				gate.reset();
				return false;
			}
		}
		return true;
	}
	
	public static void driveDist(double meters, double speed) {
		new DriveAction(DriveAction.DISTANCE, meters, speed).run();;
	}

	public static void driveTime(double millis, double speed) {
		new DriveAction(DriveAction.TIME, millis, speed).run();
	}

	public static void rotateDeg(double heading, double speed) {
		new RotateAction(heading, RotateAction.DEGREES, speed).run();
	}
	
	public static void rotateRad(double heading, double speed) {
		new RotateAction(heading, RotateAction.RADIANS, speed).run();
	}
	
	public static void waitAction(double millis) {
		new WaitAction(millis).run();
	}
}
