package org.spartabots.frc2015.action;

import org.spartabots.frc2015.util.Util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveWithJoystickAction extends Action {
	double prevTraverseMove = 0;
	boolean isBDown = false, isXDown = false;
	boolean isBDown2 = false;
	public static boolean elevatorControlEnabled = true;
	
	public DriveWithJoystickAction() {
		this.canTimeOut = false;
	}
	
	@Override
	public void init() {
		
	}

	@Override
	public boolean runPeriodic() {
		// Poll drive speed mode toggle
        // --------------------------------------------------------------------------------
		if (robot.driveController.getBButton()) {
        	if (!isBDown) {
	        	isBDown = true;
	            robot.drive.toggleSpeedMode();
        	}
        } else {
        	isBDown = false;
        }

        // --------------------------------------------------------------------------------
		if (robot.loadController.getXButton()) {
        	if (!isXDown) {
	        	isXDown = true;
	            robot.elevator.toggleSpeedMode();
        	}
        } else {
        	isXDown = false;
        }
		
		// Traverse
        // --------------------------------------------------------------------------------
		double traverseMove = -Util.cutoff(robot.driveController.getLeftTriggerAxis())
        		+ Util.cutoff(robot.driveController.getRightTriggerAxis());
		robot.drive.traverse.set(robot.drive.curveDrive(traverseMove, prevTraverseMove, true, 3, false));
		prevTraverseMove = traverseMove;
        
        // Drive
        // --------------------------------------------------------------------------------
        robot.drive.drive(
        		Util.cutoff(robot.driveController.getLeftYAxis()), 
        		Util.cutoff(robot.driveController.getRightXAxis()));
        
        // Elevator
        // --------------------------------------------------------------------------------
        if (elevatorControlEnabled) {
        	double elevatorMove = -Util.cutoff(robot.loadController.getLeftYAxis());
	        /*double elevatorMove = -Util.cutoff(robot.loadController.getLeftTriggerAxis())
	        		+ Util.cutoff(robot.loadController.getRightTriggerAxis());*/
	        robot.elevator.setElevator(elevatorMove);
        }
        
        // Clamp
        // --------------------------------------------------------------------------------
        if (robot.loadController.getLeftBumperButton() || (robot.loadController.getLeftTriggerAxis() > 0.2)) {
        	robot.profile.add(Actions.clampIn());
        } else if (robot.loadController.getRightBumperButton() || (robot.loadController.getRightTriggerAxis() > 0.2)) {
        	robot.profile.add(Actions.clampOut());
        }
        
        if (robot.loadController.getLeftTriggerAxis() > 0.2) {
        	robot.profile.add(Actions.clampOut());
        } else if (robot.loadController.getRightTriggerAxis() > 0.2) {
        	robot.profile.add(Actions.clampIn());
        }

        // Clamp
        // --------------------------------------------------------------------------------
        if (robot.driveController.getBackButton() || robot.loadController.getBackButton()) {
        	robot.drive.kpDisabled = true;
        }
        if (robot.driveController.getStartButton() || robot.loadController.getStartButton()) {
        	robot.drive.kpDisabled = false;
        }

        // Release Action
        // --------------------------------------------------------------------------------
        if (robot.loadController.getBButton()) {
            SmartDashboard.putString("ReleaseAction", "0");
        	if (!isBDown2) {
                SmartDashboard.putString("ReleaseAction", "1");
        		robot.profile.add(Actions.releaseAction());
        	}
        	isBDown2 = true;
        } else {
        	isBDown2 = false;
        }
        
		return true;
	}

	@Override
	public void done() {
		robot.drive.m_drive.arcadeDrive(0, 0);
	}

}
