package org.spartabots.frc2015.profile;

import org.spartabots.frc2015.Ports;
import org.spartabots.frc2015.Robot;
import org.spartabots.frc2015.util.Constants;
import org.spartabots.frc2015.util.Util;
import org.spartabots.frc2015.util.XboxController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ControlProfile extends Profile {
    public XboxController driveController = new XboxController(Ports.Computer.Usb0) ;
	double prevLeftX = 0;
	int i = 0;
	boolean isBDown = false;
	boolean isDrivingStraight = false;
	
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
        i++;
        SmartDashboard.putNumber("Teleop Alive", i);
        
        SmartDashboard.putNumber("Gyro Angle", robot.drive.getGyroAngle());
        SmartDashboard.putNumber("Acceleration X", robot.drive.getAccelX());
        SmartDashboard.putNumber("Acceleration Y", robot.drive.getAccelY());
        SmartDashboard.putNumber("Acceleration Z", robot.drive.getAccelZ());
        SmartDashboard.putNumber("Left Encoder Raw", robot.drive.leftEc.getRaw());
        SmartDashboard.putNumber("Right Encoder Distance", robot.drive.getRightEncoderDistance());
        SmartDashboard.putNumber("Left Encoder Distance", robot.drive.getLeftEncoderDistance());
		
		double traverseMove = -Util.cutoff(driveController.getLeftTriggerAxis()) + Util.cutoff(driveController.getRightTriggerAxis());
        double leftY = -Util.cutoff(driveController.getLeftYAxis());
        double rightX = Util.cutoff(driveController.getRightXAxis());
        
        if (rightX == 0 && !isDrivingStraight)
        	isDrivingStraight = true;
        else if (rightX != 0)
        	isDrivingStraight = false;
        
        if (isDrivingStraight) {
        	double angle = robot.drive.getGyroAngle();
        	robot.drive.drive(1, -angle * Constants.GYRO_KP);
        }
        
        // Traversing wheel
        double curve = robot.drive.curveDrive(traverseMove, prevLeftX, true, 3);
        SmartDashboard.putNumber("Curve Drive", curve);
        robot.drive.traverse.set(curve);
        
        // Speedmode (slow vs fast, caps max speed)
        if (driveController.getBButton()) {
        	if (!isBDown) {
	        	isBDown = true;
	            robot.drive.toggleSpeedMode();
        	}
        } else {
        	isBDown = false;
        }
        
        // Drives robot with curves
        robot.drive.drive(leftY, rightX);
        
        prevLeftX = traverseMove;
	}
	
	@Override
	public void done() {
		
	}
}
