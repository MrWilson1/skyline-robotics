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
	private static Profile current;
	public boolean running = true;
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
		if (Profile.current != null && Profile.current.isRunning()) {
			Profile.current.running = false;
		}
		
		this.running = true;
		Profile.current = this;
		
		init();
		
		if (skipPeriodic) return;
		
		if (mode == CONTROL) {
			while (running && robot.isOperatorControl() && robot.isEnabled()) {
				this.controlPeriodic();
				Timer.delay(0.005);
			}
		} else if (mode == AUTONOMOUS) {
			while (running && robot.isAutonomous() && robot.isEnabled()) {
				this.autoPeriodic();
				Timer.delay(0.005);
			}
		} else if (mode == TEST) {
			while (running && robot.isTest() && robot.isEnabled()) {
				this.testPeriodic();
				Timer.delay(0.005);
			}
		}
		done();
		this.running = false;
	}
	
	public boolean isRunning() {
		if (!running)
			return false;
		if (!robot.isEnabled())
			return false;
		
		if (mode == CONTROL) {
			return robot.isOperatorControl();
		} else if (mode == AUTONOMOUS) {
			return robot.isAutonomous();
		} else if (mode == TEST) {
			return robot.isEnabled();
		} else {
			return false;
		}
	}
	
	public static Profile getCurrent() {
		return current;
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
