package org.spartabots.frc2015.action;

public class Actions {
	
	public static void driveDist(double meters, double speed) {
		new DriveAction(DriveAction.DISTANCE, meters, speed).run();
	}

	public static void driveTime(double millis, double speed) {
		new DriveAction(DriveAction.TIME, millis, speed).run();
	}

	public static void travserseDist(double meters, double speed) {
		new TraverseAction(TraverseAction.DISTANCE, meters, speed).run();
	}

	public static void traverseTime(double millis, double speed) {
		new TraverseAction(TraverseAction.TIME, millis, speed).run();
	}

	public static void rotateDeg(double heading, double speed) {
		new RotateAction(heading, RotateAction.DEGREES, speed).run();
	}
	
	public static void rotateRad(double heading, double speed) {
		new RotateAction(heading, RotateAction.RADIANS, speed).run();
	}
	
	public static void elevator(double power, double timeMillis) {
		new ElevatorAction(power, timeMillis).run();
	}
	
	public static void waitAction(double millis) {
		new WaitAction(millis).run();
	}

	public static void clampIn() {
		new ClampAction(ClampAction.IN).run();
	}

	public static void clampOut() {
		new ClampAction(ClampAction.OUT).run();
	}
}
