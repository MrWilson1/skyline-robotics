package org.spartabots.frc2015.action;

/**
 * A factory class for Actions
 */
public class Actions {
	
	public static DriveAction driveDist(double meters, double speed) {
		return new DriveAction(DriveAction.DISTANCE, meters, speed);
	}

	public static DriveAction driveTime(double millis, double speed) {
		return new DriveAction(DriveAction.TIME, millis, speed);
	}

	public static TraverseAction travserseDist(double meters, double speed) {
		return new TraverseAction(TraverseAction.DISTANCE, meters, speed);
	}

	public static TraverseAction traverseTime(double millis, double speed) {
		return new TraverseAction(TraverseAction.TIME, millis, speed);
	}

	public static RotateAction rotateDeg(double heading, double speed) {
		return new RotateAction(heading, RotateAction.DEGREES, speed);
	}
	
	public static RotateAction rotateRad(double heading, double speed) {
		return new RotateAction(heading, RotateAction.RADIANS, speed);
	}
	
	public static ElevatorAction elevatorTime(double power, double timeMillis) {
		return new ElevatorAction(power, timeMillis);
	}
	
	public static ElevatorAction elevator(double power) {
		return new ElevatorAction(power, -1);
	}
	
	public static WaitAction waitAction(double millis) {
		return new WaitAction(millis);
	}

	public static ClampAction clampIn() {
		return new ClampAction(ClampAction.IN);
	}

	public static ClampAction clampOut() {
		return new ClampAction(ClampAction.OUT);
	}
}
