package org.spartabots.frc2015.profile;

import org.spartabots.frc2015.action.Actions;
import org.spartabots.frc2015.auto.Auto1Bin;
import org.spartabots.frc2015.auto.Auto1Tote;
import org.spartabots.frc2015.auto.Auto2Tote;
import org.spartabots.frc2015.auto.Auto3Tote;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoProfile extends Profile {
	int selectedAction = 1;
	
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
		
		switch (selectedAction) {
		case 0: add(Actions.waitAction(1000));
			break;
		case 1: add(new Auto1Tote());
			break;
		case 2: add(new Auto2Tote());
			break;
		case 3: add(new Auto3Tote());
			break;
		case 4: add(new Auto1Bin());
			break;
		case 5: add(Actions.driveDist(8.5, 0.85));
			break;
		default:
			break;
		}
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
