package org.spartabots.frc2015.action;

public class ElevatorAction extends Action {
	double power;
	
	public ElevatorAction(double power, double timeMillis) {
		this.power = power;
		this.setTimeout(timeMillis);
	}
	
	@Override
	public void init() {
		robot.elevator.setElevator(power);
	}

	@Override
	public boolean running() {
		robot.elevator.setElevator(power);
		return true;
	}

	@Override
	public void done() {
		robot.elevator.setElevator(0);
	}

}
