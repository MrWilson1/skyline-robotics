package org.spartabots.frc2015.profile;

import org.spartabots.frc2015.action.DriveWithJoystickAction;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ControlProfile extends Profile {
	
	public ControlProfile() {
		super(Profile.CONTROL);
	}
	
	@Override
	public void init() {
		robot.drive.resetGyro();
		robot.drive.resetEncoders();
		
		add(new DriveWithJoystickAction());
	}
	
	@Override
	public void log() {
        SmartDashboard.putNumber("Gyro Angle", robot.drive.getGyroAngle());
        SmartDashboard.putBoolean("Is driving straight", robot.drive.isDrivingStraight);
        SmartDashboard.putNumber("Belt Encoder Raw", robot.drive.beltEc.getRaw());
        SmartDashboard.putNumber("Belt Encoder Distance", robot.drive.getBeltEncoderDistance());
        SmartDashboard.putNumber("Traverse Encoder Raw", robot.drive.traverseEc.getRaw());
        SmartDashboard.putNumber("Traverse Encoder Distance", robot.drive.getTraverseEncoderDistance());
        SmartDashboard.putNumber("Elevator Encoder Raw", robot.elevator.ec.getRaw());
        SmartDashboard.putNumber("Elevator Encoder Distance", robot.elevator.getDistanceFromGround());
        
        SmartDashboard.putBoolean("Is drive speed mode", robot.drive.isSpeedMode());
        SmartDashboard.putBoolean("Is elevator speed mode", robot.elevator.isSpeedMode());

        SmartDashboard.putNumber("Accel X", robot.drive.getAccelX());
        SmartDashboard.putNumber("Accel Y", robot.drive.getAccelY());
        SmartDashboard.putNumber("Accel Z", robot.drive.getAccelZ());
	}
	
}
