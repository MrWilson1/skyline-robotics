package org.spartabots.frc2015.mode;

import org.spartabots.frc2015.Robot;
import org.spartabots.frc2015.Util;

public class ControlMode extends Mode implements Runnable {
	double prevLeftX = 0;
	
	public ControlMode(Robot robot) {
		super(robot, Mode.CONTROL);
	}

	@Override
	public void controlPeriodic() {
        double leftX = Util.cutoff(robot.driveController.getLeftXAxis());
        double leftY = Util.cutoff(robot.driveController.getLeftYAxis());
        double rightX = Util.cutoff(robot.driveController.getRightXAxis());
        boolean bButton = robot.driveController.getBButton();
        
        // Traversing wheel
        robot.traverse.set(robot.drive.curveDrive(leftX, prevLeftX, true, 3));
        
        // Speedmode (slow vs fast, caps max speed)
        robot.drive.speedmode(bButton);
        
        // Drives robot with curves
        robot.drive.move(leftY, rightX);
        
        prevLeftX = leftX;
	}

}
