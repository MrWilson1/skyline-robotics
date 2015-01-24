package org.spartabots.frc2015.mode;

import org.spartabots.frc2015.Ports;
import org.spartabots.frc2015.Robot;
import org.spartabots.frc2015.util.Util;
import org.spartabots.frc2015.util.XboxController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ControlMode extends Mode {
    public XboxController driveController = new XboxController(Ports.Computer.Usb0) ;
	double prevLeftX = 0;
	int i = 0;
	boolean isBDown = false;
	
	public ControlMode(Robot robot) {
		super(robot, Mode.CONTROL);
	}
	
	@Override
	public void init() {
		
	}
	
	@Override
	public void controlPeriodic() {
        i++;
        SmartDashboard.putNumber("Teleop Alive", i);
        SmartDashboard.putNumber("Gyro Angle", robot.drive.getGyroAngle());
		
		double leftX = Util.cutoff(driveController.getLeftXAxis());
        double leftY = Util.cutoff(driveController.getLeftYAxis());
        double rightX = Util.cutoff(driveController.getRightXAxis());
        
        // Traversing wheel
        double curve = robot.drive.curveDrive(leftX, prevLeftX, true, 3);
        SmartDashboard.putNumber("Curve Drive", curve);
        robot.drive.traverse.set(curve);
        
        // Speedmode (slow vs fast, caps max speed)
        if (driveController.getBButton()) {
        	if (isBDown) {
	        	isBDown = true;
	            robot.drive.toggleSpeedMode();
        	}
        } else {
        	isBDown = false;
        }
        
        // Drives robot with curves
        robot.drive.drive(leftY, rightX);
        
        prevLeftX = leftX;
	}
	
	@Override
	public void done() {
		
	}
}
