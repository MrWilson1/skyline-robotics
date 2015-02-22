package org.spartabots.frc2015.action;

import org.spartabots.frc2015.util.Constants;

/**
 * A factory class for Actions
 */
public class Actions {
	
	public static DriveAction driveDist(double feet, double speed, boolean resetEncoders) {
		return new DriveAction(DriveAction.DISTANCE, feet, speed, resetEncoders);
	}

	public static DriveAction driveTime(double millis, double speed, boolean resetEncoders) {
		return new DriveAction(DriveAction.TIME, millis, speed, resetEncoders);
	}

	public static TraverseAction traverseDist(double feet, double speed, boolean resetEncoders) {
		return new TraverseAction(TraverseAction.DISTANCE, feet, speed, resetEncoders);
	}

	public static TraverseAction traverseTime(double millis, double speed, boolean resetEncoders) {
		return new TraverseAction(TraverseAction.TIME, millis, speed, resetEncoders);
	}

	public static RotateAction rotateDeg(double heading, double speed, boolean resetHeading) {
		return new RotateAction(heading, RotateAction.DEGREES, speed, resetHeading);
	}
	
	public static RotateAction rotateRad(double heading, double speed, boolean resetHeading) {
		return new RotateAction(heading, RotateAction.RADIANS, speed, resetHeading);
	}
	
	public static ElevatorAction elevatorTime(double power, double timeMillis) {
		return new ElevatorAction(power, timeMillis);
	}
	
	public static ElevatorAction elevator(double power) {
		return new ElevatorAction(power, -1);
	}
	
	public static ElevatorGotoAction elevatorGoto(double feetFromGround) {
		return new ElevatorGotoAction(feetFromGround);
	}
	
	public static ElevatorGotoAction elevatorGotoBottom() {
		return new ElevatorGotoAction(0);
	}
	
	public static ElevatorGotoAction elevatorGotoTop() {
		return new ElevatorGotoAction(Constants.ELEVATOR_HEIGHT);
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

	public static RumbleAction rumbleLeft(float value, double time) {
		return new RumbleAction(RumbleAction.RUMBLE_LEFT, value, time);
	}

	public static RumbleAction rumbleRight(float value, double time) {
		return new RumbleAction(RumbleAction.RUMBLE_RIGHT, value, time);
	}

	public static RumbleAction rumbleBoth(float value, double time) {
		return new RumbleAction(RumbleAction.RUMBLE_BOTH, value, time);
	}

	public static ReleaseAction releaseAction() {
		return new ReleaseAction();
	}
}
