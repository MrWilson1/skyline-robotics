package org.spartabots.frc2015.profile;

import org.spartabots.frc2015.Ports;
import org.spartabots.frc2015.Robot;
import org.spartabots.frc2015.action.Actions;
import org.spartabots.frc2015.util.Util;
import org.spartabots.frc2015.util.XboxController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ControlProfile extends Profile {
    public XboxController driveController = new XboxController(Ports.Computer.Usb0) ;
	int i = 0;
	boolean isBDown = false;
	double prevTraverseMove = 0;
	
	public ControlProfile(Robot robot) {
		super(Profile.CONTROL, robot);
	}
	
	@Override
	public void init() {
		robot.drive.resetGyro();
		robot.drive.resetEncoders();
	}
	
	@Override
	public void controlPeriodic() {
		log();
		pollSpeedMode();
		
        // Traverse
        // --------------------------------------------------------------------------------
		double traverseMove = -Util.cutoff(driveController.getLeftTriggerAxis())
        		+ Util.cutoff(driveController.getRightTriggerAxis());
		robot.drive.traverse.set(robot.drive.curveDrive(traverseMove, prevTraverseMove, true, 3));
		prevTraverseMove = traverseMove;
        
        // Drive
        // --------------------------------------------------------------------------------
        robot.drive.drive(
        		-Util.cutoff(driveController.getLeftYAxis()), 
        		Util.cutoff(driveController.getRightXAxis()));
        
        // Elevator
        // --------------------------------------------------------------------------------
        if (driveController.getLeftBumperButton()) { // Elevator down
        	//Actions.elevator(-1).run();
        } else if (driveController.getRightBumperButton()) { // Elevator up
        	//Actions.elevator(1).run();
        } else { // Elevator stop
        	//Actions.elevator(0).run();
        }
        
        // Clamp
        // --------------------------------------------------------------------------------
        if (driveController.getAButton()) {
        	Actions.clampOut().run();
        } else if (driveController.getAButton()) {
        	Actions.clampIn().run();
        }
	}
	
	public void pollSpeedMode() {
		 // Speedmode (slow vs fast, caps max speed)
        if (driveController.getBButton()) {
        	if (!isBDown) {
	        	isBDown = true;
	            robot.drive.toggleSpeedMode();
        	}
        } else {
        	isBDown = false;
        }
	}
	
	public void log() {
        i++;
        SmartDashboard.putNumber("Teleop Alive", i);
        SmartDashboard.putNumber("Gyro Angle", robot.drive.getGyroAngle());
        SmartDashboard.putNumber("Acceleration X", robot.drive.getAccelX());
        SmartDashboard.putNumber("Acceleration Y", robot.drive.getAccelY());
        SmartDashboard.putNumber("Acceleration Z", robot.drive.getAccelZ());
        SmartDashboard.putNumber("Left Encoder Raw", robot.drive.leftEc.getRaw());
        SmartDashboard.putNumber("Right Encoder Distance", robot.drive.getRightEncoderDistance());
        SmartDashboard.putNumber("Left Encoder Distance", robot.drive.getLeftEncoderDistance());
	}
	
	@Override
	public void done() {
		
	}
}
