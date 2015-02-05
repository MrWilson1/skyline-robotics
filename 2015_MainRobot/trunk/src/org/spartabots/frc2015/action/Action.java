package org.spartabots.frc2015.action;

import org.spartabots.frc2015.Robot;
import org.spartabots.frc2015.profile.Profile;

import edu.wpi.first.wpilibj.Timer;

/**
 * Action<br/></br>
 * 
 * The init() method is called once to perform the action. The action will continue
 * to be performed until either the action times out or the action decides it should
 * stop running by the isRunning method. Once the action is complete, the done method
 * will be called stop and cleanup whatever needs to be stopped or cleaned up.
 */
public abstract class Action {
	Timer timer = new Timer();
	boolean done = false;
	double timeout = 100; // in seconds
	Robot robot = Robot.getInstance();
	
	public abstract void init();
	/**
	 * Whether or not to continue performing this action. The super class {@link Running}
	 * will automatically handle isTimedOut. If timing out should be the only way to stop
	 * the action, you may just return true on this method.
	 *  
	 * @return running
	 */
	public abstract boolean running();
	public abstract void done();
	
	public void cancel() {
		done = true;
	}
	
	public void run() {
		run(null);
	}
	
	public void run(ActionThread actionThread) {
		Profile.getCurrent().actionRegister(this);
		timer.start();
		init();
		if (!done) {
			while (!done && running() && !isTimedOut()) {
				Timer.delay(0.005);
			}
		}
		timer.stop();
		done();
		this.robot = null;
		if (actionThread != null)
			actionThread.actionDone();
		Profile.getCurrent().actionDone(this);
	}
	
	public boolean isDone() {
		return done;
	}
	
	public boolean isTimedOut() {
		return timer.get() > timeout;
	}
	
	/**
	 * Get time the action was running
	 * 
	 * @return time in seconds
	 */
	public double getTime() {
		return timer.get();
	}
	
	public void setTimeout(double milliseconds) {
		this.timeout = (milliseconds / 1000.0D);
	}
}
