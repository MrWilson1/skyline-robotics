package org.spartabots.frc2015.action;

import org.spartabots.frc2015.util.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorGotoAction extends Action {
	int direction; // -1 if going down, 0 if going no where, 1 if going up
	final double dfg;
	double error = 0.1;
	
	/**
	 * Go to a certain number of feet from ground.
	 * 
	 * @param feetFromGround Feet from ground to move to. If less than current
	 * elevator position, elevator will go down. If greater than current position,
	 * elevator will go up. Max feet from ground is {@link Constants#ELEVATOR_HEIGHT}
	 */
	public ElevatorGotoAction(double feetFromGround) {
		this.canTimeOut = true;
		this.setTimeout(6500);
		if (feetFromGround < 0)
			feetFromGround = 0;
		if (feetFromGround > Constants.ELEVATOR_HEIGHT)
			feetFromGround = Constants.ELEVATOR_HEIGHT;
		double newDfg = feetFromGround;
		
		SmartDashboard.putString("Whatcha doing ElevatorGoto?", "init");
		
		double c_dfg = robot.elevator.getDistanceFromGround();
		if (newDfg > c_dfg) {
			direction = 1;
		} else if (newDfg < c_dfg) {
			direction = -1;
		} else {
			direction = 0;
		}
		
		this.dfg = newDfg;

		SmartDashboard.putString("ElevatorGoto init", dfg + ", " + direction);
	}
	
	@Override
	public void init() {
		if (direction == 0) {
			SmartDashboard.putString("Whatcha doing ElevatorGoto?", "cancelled out, direction");
			cancel();
		}
	}

	@Override
	public boolean runPeriodic() {
		if (dfg == 0) {
			robot.elevator.setElevator(-1);
			return !robot.elevator.isAtBottom();
		} else if (dfg == Constants.ELEVATOR_HEIGHT) {
			robot.elevator.setElevator(1);
			return !robot.elevator.isAtTop();
		}
		
		double c_dfg = robot.elevator.getDistanceFromGround();
		
		SmartDashboard.putString("Whatcha doing ElevatorGoto?", "moving, c=" + c_dfg + ", to="+dfg);
		
		if (dfg < c_dfg-error) {
			robot.elevator.setElevator(-0.8);
		} else if (c_dfg+error < dfg) {
			robot.elevator.setElevator(0.8);
		} else {
			return false;
		}
		return true;
	}

	@Override
	public void done() {
		SmartDashboard.putString("Whatcha doing ElevatorGoto?", "done");
		robot.elevator.setElevator(0);
	}
	
}
