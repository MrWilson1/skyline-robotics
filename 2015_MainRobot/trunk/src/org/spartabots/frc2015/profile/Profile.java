package org.spartabots.frc2015.profile;

import java.util.ArrayList;
import java.util.function.Supplier;

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
	public static final Robot robot = Robot.getInstance();
	private final int mode;
	public boolean running = true;
	
	// Mode Types
	public static final int CONTROL = 0;
	public static final int AUTONOMOUS = 1;
	public static final int TEST = 2;
	
	// Actions collections. There should only ever be one profile running at a time,
	// so these collections are static.
	protected static ArrayList<Action> actions = new ArrayList<Action>(100);
	protected static ArrayList<Action> toRemove = new ArrayList<Action>(50);
	protected static ArrayList<Action> toAdd = new ArrayList<Action>(50);
	
	public Profile(int mode) {
		this.mode = mode;
	}
	
	public final void start() {
		if (mode == CONTROL) {
			start(robot::isOperatorControl);
		} else if (mode == AUTONOMOUS) {
			start(robot::isAutonomous);
		} else if (mode == TEST) {
			start(robot::isTest);
		}
	}
	
	public final void start(Supplier<Boolean> modeCondition) {
		this.running = true;
		if (robot.profile != null && robot.profile.running)
			robot.profile.running = false;
		robot.profile = this;
		init();
		
		while (running && robot.isEnabled() && modeCondition.get()) {
			actions.addAll(toAdd);
			toAdd.clear();
			actions.removeAll(toRemove);
			toRemove.clear();
			
			for (int i = 0; i < actions.size(); i++) {
				Action a = actions.get(i);
				if (!a.running())
					toRemove.add(a);
			}
			
			log();
			
			Timer.delay(0.005);
		}
		cleanup();
		done();
		this.running = false;
	}
	
	public void stop() {
		this.running = false;
	}
	
	public static void add(Action a) {
		toAdd.add(a);
		a.init();
	}
	
	public static void remove(Action a) {
		toRemove.add(a);
	}
	
	public static void cancelActions() {
		for (int i = 0; i < actions.size(); i++) {
			Action a = actions.get(i);
			if (!a.isDone()) {
				a.cancel();
			}
		}
	}
	
	protected static void cleanup() {
		cancelActions();
		actions.clear();
		toRemove.clear();
		toAdd.clear();
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
