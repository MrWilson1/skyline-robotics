package org.spartabots.frc2015.profile;

import java.util.ArrayList;

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
	ArrayList<Action> actions = new ArrayList<Action>();
	ArrayList<Action> toRemove = new ArrayList<Action>();
	ArrayList<Action> toAdd = new ArrayList<Action>();
	private final int mode;
	public boolean running = true;
	Robot robot;
	
	public Profile(int mode, Robot robot) {
		this.mode = mode;
		this.robot = robot;
	}
	
	public final void start() {
		this.robot.profile = this;
		init();
		
		while (running && robot.isEnabled()) {
			if (mode == CONTROL) {
				if (!robot.isOperatorControl()) break;
			} else if (mode == AUTONOMOUS) {
				if (!robot.isAutonomous()) break;
			} else if (mode == TEST) {
				if (!robot.isTest()) break;
			}
			
			actions.addAll(toAdd);
			toAdd.clear();
			actions.removeAll(toRemove);
			toRemove.clear();
			
			for (Action a : actions) {
				if (!a.running())
					toRemove.add(a);
			}
			
			log();
			
			Timer.delay(0.005);
		}
		
		done();
		this.running = false;
	}
	
	public void add(Action a) {
		toAdd.add(a);
		a.init();
	}
	
	public void remove(Action a) {
		toRemove.add(a);
	}
	
	/** Whether or not the Profile is currently running */
	public boolean isRunning() { return running; }
	/** Log interesting values to the SmartDashhboard */
	public void log() {}
	/** Init method */
	public void init() {}
	/** Called when Profile is done running */
	public void done() {}
}
