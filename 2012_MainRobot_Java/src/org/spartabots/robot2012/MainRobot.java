
package org.spartabots.robot2012;

import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SimpleRobot;
import edu.wpi.first.wpilibj.Timer;
import org.spartabots.robot2012.controller.XboxController;

public class MainRobot extends SimpleRobot {
    RobotDrive m_drive;
    XboxController driveController;
    Jaguar topLeftShooter;
    Jaguar topRightShooter;
    Jaguar bottomLeftShooter;
    Jaguar bottomRightShooter;
    Jaguar elevator;
    Shooter shooter;
    
    public MainRobot() {
        super();
        initializeHardware();
        initializeSoftware();
    }

    private void initializeHardware() {
	m_drive                 = new RobotDrive(Ports.DigitalSidecar.Pwm8, Ports.DigitalSidecar.Pwm10);
	driveController 	= new XboxController(Ports.Computer.Usb1);
	topLeftShooter 		= new Jaguar(Ports.DigitalSidecar.Pwm1);
	topRightShooter 	= new Jaguar(Ports.DigitalSidecar.Pwm2);
	bottomLeftShooter 	= new Jaguar(Ports.DigitalSidecar.Pwm3);
	bottomRightShooter 	= new Jaguar(Ports.DigitalSidecar.Pwm4);
	elevator 		= new Jaguar(Ports.DigitalSidecar.Pwm5);
    }

    private void initializeSoftware() {
	shooter = new Shooter(topLeftShooter, topRightShooter, bottomLeftShooter, bottomRightShooter);
    }
    
    public void autonomous() {
        m_drive.setSafetyEnabled(false);
        m_drive.drive(0.5, 0);
        Timer.delay(2);
        m_drive.drive(0, 0);
    }
    
    public void operatorControl() {
        m_drive.setSafetyEnabled(true);
        while (this.isOperatorControl() && this.isEnabled()) {
            double leftY = -Util.cutoff(driveController.getLeftXAxis());
            double rightY = -Util.cutoff(driveController.getRightYAxis());
            
            m_drive.tankDrive(leftY, rightY);
            
            boolean lBumper = driveController.getLeftBumperButton();
            boolean rBumper = driveController.getRightBumperButton();
            if (rBumper) {
                // Move loader upwards (take in ball)
                elevator.set(-1);
            } else if (lBumper) {
                // Move loader downwards (put out ball)
                elevator.set(1);
            } else {
                elevator.set(0.0);
            }

            double triggerY = Util.cutoff(driveController.getTriggerAxis());
            if (triggerY < -0.4) {
                shooter.setSpeed(0.5f);
            } else {
                shooter.setSpeed(0);
            }
            
            Timer.delay(0.005); // This should be the last line in the loop
        }
    }
    
    public void test() {
    
    }
}