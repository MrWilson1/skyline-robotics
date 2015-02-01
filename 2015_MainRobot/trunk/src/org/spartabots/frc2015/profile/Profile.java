package org.spartabots.frc2015.profile;

import org.spartabots.frc2015.Robot;

import edu.wpi.first.wpilibj.Timer;

/*
 *  SAFETY PIG HAS ARRIVED!
 * 
 *   _._ _..._ .-',     _.._(`))
 *  '-. `     '  /-._.-'    ',/
 *     )         \            '.
 *    / _    _    |             \
 *   |  a    a    /              |
 *   \   .-.                     ;  
 *    '-('' ).-'       ,'       ;
 *       '-;           |      .'
 *          \           \    /
 *          | 7  .__  _.-\   \
 *          | |  |  ``/  /`  /
 *         /,_|  |   /,_/   /
 *            /,_/      '`-'
 * 
 */
public abstract class Profile {
	public static final int CONTROL = 0;
	public static final int AUTONOMOUS = 1;
	public static final int TEST = 2;
	private final int mode;
	private boolean skipPeriodic = false;
	Robot robot;
	
	public Profile(int mode, Robot robot) {
		this.mode = mode;
		this.robot = robot;
	}
	
	public Profile(int mode, Robot robot, boolean skipPeriodic) {
		this.mode = mode;
		this.robot = robot;
		this.skipPeriodic = skipPeriodic;
	}
	
	public final void start() {
		if (skipPeriodic) {
			init();
			done();
			return;
		}
		
		if (mode == CONTROL) {
			init();
			while (robot.isOperatorControl() && robot.isEnabled()) {
				this.controlPeriodic();
				Timer.delay(0.005);
			}
			done();
		} else if (mode == AUTONOMOUS) {
			init();
			while (robot.isAutonomous() && robot.isEnabled()) {
				this.autoPeriodic();
				Timer.delay(0.005);
			}
			done();
		} else if (mode == TEST) {
			init();
			while (robot.isTest() && robot.isEnabled()) {
				this.testPeriodic();
				Timer.delay(0.005);
			}
			done();
		}
	}
	
	public void init() {
		
	}
	
	public void done() {
		
	}

	public void controlPeriodic() {
	}

	public void autoPeriodic() {
	}
	
	public void testPeriodic() {
	}
}