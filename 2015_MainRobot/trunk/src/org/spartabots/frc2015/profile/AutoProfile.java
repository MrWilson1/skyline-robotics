package org.spartabots.frc2015.profile;

import org.spartabots.frc2015.action.Actions;
import org.spartabots.frc2015.auto.*;

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
		case 6: add(new AutoTest());
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
        SmartDashboard.putNumber("Belt Encoder Raw", robot.drive.beltEc.getRaw());
        SmartDashboard.putNumber("Belt Encoder Distance", robot.drive.getBeltEncoderDistance());
        SmartDashboard.putNumber("Traverse Encoder Raw", robot.drive.traverseEc.getRaw());
        SmartDashboard.putNumber("Traverse Encoder Distance", robot.drive.getTraverseEncoderDistance());
	}
	
}
