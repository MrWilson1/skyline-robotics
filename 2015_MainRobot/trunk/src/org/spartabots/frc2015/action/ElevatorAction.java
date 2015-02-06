package org.spartabots.frc2015.action;

public class ElevatorAction extends Action {
	double power;
	boolean timeLimit = true;
	
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
		robot.elevator.setElevator(power);
		if (timeLimit)
			done();
	}

	@Override
	public boolean runPeriodic() {
		return true;
	}

	@Override
	public void done() {
		if (this.timeLimit)
			robot.elevator.setElevator(0);
	}
	
}
