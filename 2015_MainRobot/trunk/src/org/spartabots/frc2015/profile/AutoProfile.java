package org.spartabots.frc2015.profile;

import org.spartabots.frc2015.action.Action;
import org.spartabots.frc2015.action.Actions;
import org.spartabots.frc2015.auto.Auto1Bin;
import org.spartabots.frc2015.auto.Auto1Tote;
import org.spartabots.frc2015.auto.Auto2Tote;
import org.spartabots.frc2015.auto.Auto3Tote;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoProfile extends Profile {
	int selectedAction = 1;
	Action[] autoActions = {
			Actions.waitAction(1000),
			new Auto1Tote(),
			new Auto2Tote(),
			new Auto3Tote(),
			new Auto1Bin(),
			Actions.driveDist(8.5, 0.85)
		};
	
	public AutoProfile() {
		super(Profile.AUTONOMOUS);
	}
	
	public void setSelectedAction(int selectedAction) {
		this.selectedAction = selectedAction;
	}
	
	@Override
	public void init() {
		robot.drive.resetGyro();
		robot.drive.resetEncoders();
		
		if (selectedAction < autoActions.length)
			add(autoActions[selectedAction]);
	}
	
	@Override
	public void log() {
        SmartDashboard.putNumber("Gyro Angle", robot.drive.getGyroAngle());
        SmartDashboard.putNumber("Acceleration X", robot.drive.getAccelX());
        SmartDashboard.putNumber("Acceleration Y", robot.drive.getAccelY());
        SmartDashboard.putNumber("Acceleration Z", robot.drive.getAccelZ());
        SmartDashboard.putNumber("Left Encoder Raw", robot.drive.leftEc.getRaw());
        SmartDashboard.putNumber("Right Encoder Distance", robot.drive.getRightEncoderDistance());
        SmartDashboard.putNumber("Left Encoder Distance", robot.drive.getLeftEncoderDistance());
	}
	
}
