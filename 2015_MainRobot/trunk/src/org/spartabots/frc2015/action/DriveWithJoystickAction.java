package org.spartabots.frc2015.action;

import org.spartabots.frc2015.profile.Profile;
import org.spartabots.frc2015.util.Util;

public class DriveWithJoystickAction extends Action {
	double prevTraverseMove = 0;
	boolean isBDown = false;
	
	public DriveWithJoystickAction() {
		this.canTimeOut = false;
	}
	
	@Override
	public void init() {
		
	}

	@Override
	public boolean runPeriodic() {
		// Poll drive speed mode
		if (robot.driveController.getBButton()) {
        	if (!isBDown) {
	        	isBDown = true;
	            robot.drive.toggleSpeedMode();
        	}
        } else {
        	isBDown = false;
        }
		
		// Traverse
        // --------------------------------------------------------------------------------
		double traverseMove = -Util.cutoff(robot.driveController.getLeftTriggerAxis())
        		+ Util.cutoff(robot.driveController.getRightTriggerAxis());
		robot.drive.traverse.set(robot.drive.curveDrive(traverseMove, prevTraverseMove, true, 3));
		prevTraverseMove = traverseMove;
        
        // Drive
        // --------------------------------------------------------------------------------
        robot.drive.drive(
        		-Util.cutoff(robot.driveController.getLeftYAxis()), 
        		Util.cutoff(robot.driveController.getRightXAxis()));
        
        // Elevator
        // --------------------------------------------------------------------------------
        double elevatorMove= -Util.cutoff(robot.driveController2.getLeftTriggerAxis())
        		+ Util.cutoff(robot.driveController2.getRightTriggerAxis());
        robot.elevator.setElevator(elevatorMove);
        
        // Clamp
        // --------------------------------------------------------------------------------
        if (robot.driveController2.getLeftBumperButton()) {
        	Profile.add(Actions.clampOut());
        } else if (robot.driveController2.getRightBumperButton()) {
        	Profile.add(Actions.clampIn());
        }
        
		return true;
	}

	@Override
	public void done() {
		robot.drive.m_drive.arcadeDrive(0, 0);
	}

}
