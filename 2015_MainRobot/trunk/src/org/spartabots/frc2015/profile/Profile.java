package org.spartabots.frc2015.profile;

import java.util.ArrayList;
import java.util.HashSet;

import org.spartabots.frc2015.Robot;
import org.spartabots.frc2015.action.Action;

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
	HashSet<Action> actionList = new HashSet<Action>();
	ArrayList<Action> actionListToAdd = new ArrayList<Action>();
	ArrayList<Action> actionListToRemove = new ArrayList<Action>();
	
	
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
		
		if (skipPeriodic) {
			init();
			cleanup();
			return;
		}
		
		if (mode == CONTROL) {
			init();
			while (running && robot.isOperatorControl() && robot.isEnabled()) {
				this.actionList.addAll(this.actionListToAdd);
				this.actionList.removeAll(this.actionListToRemove);
				this.actionListToAdd.clear();
				this.actionListToRemove.clear();
				
				this.controlPeriodic();
				Timer.delay(0.005);
			}
			cleanup();
		} else if (mode == AUTONOMOUS) {
			init();
			while (running && robot.isAutonomous() && robot.isEnabled()) {
				this.autoPeriodic();
				Timer.delay(0.005);
			}
			cleanup();
		} else if (mode == TEST) {
			init();
			while (running && robot.isTest() && robot.isEnabled()) {
				this.testPeriodic();
				Timer.delay(0.005);
			}
			cleanup();
		}
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
	
	protected void cleanup() {
		for (Action a : actionList)
			a.cancel();
		actionList.clear();
		done();
	}
	
	public void done() {
	}

	public void controlPeriodic() {
	}

	public void autoPeriodic() {
	}
	
	public void testPeriodic() {
	}
	
	public void actionRegister(Action action) {
		this.actionListToAdd.add(action);
	}
	
	public void actionDone(Action action) {
		this.actionListToRemove.add(action);
	}
}
