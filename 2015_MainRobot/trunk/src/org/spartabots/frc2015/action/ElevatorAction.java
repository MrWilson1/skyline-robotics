package org.spartabots.frc2015.action;

public class ElevatorAction extends Action {
	double power;
	
	/**
	 * 
	 * @param power Power to move at
	 * @param timeLimit how long to move for (optional), set to -1 if automatic cancellation not necessarys
	 */
	public ElevatorAction(double power, double timeLimit) {
		this.power = power;
		this.setTimeout(timeLimit);
		if (timeLimit < 0)
			this.canTimeOut = false;
	}
	
	@Override
	public void init() {
		DriveWithJoystickAction.elevatorControlEnabled = false;
        robot.elevator.setElevator(power);
		if (!this.canTimeOut)
			cancel();
	}

	@Override
	public boolean runPeriodic() {
		robot.elevator.setElevator(power);
		return true;
	}

	@Override
	public void done() {
		DriveWithJoystickAction.elevatorControlEnabled = true;
		if (this.canTimeOut)
			robot.elevator.setElevator(0);
	}
	
}
