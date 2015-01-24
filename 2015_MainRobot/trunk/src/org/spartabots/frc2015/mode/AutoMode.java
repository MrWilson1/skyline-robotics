package org.spartabots.frc2015.mode;

import org.spartabots.frc2015.Robot;
import org.spartabots.frc2015.subsystem.Drive;

public class AutoMode extends Mode {

	public AutoMode(Robot robot) {
		super(robot, Mode.AUTONOMOUS);
	}
	
	@Override
	public void init() {
		robot.drive.resetGyro();
	}

	@Override
	public void autoPeriodic() {
		double angle = robot.drive.getGyroAngle();
		robot.drive.m_drive.arcadeDrive(robot.drive.curveDrive(1, 1, true, 3), -angle * Drive.kp);
	}
	
	@Override
	public void done() {
		
	}

}
