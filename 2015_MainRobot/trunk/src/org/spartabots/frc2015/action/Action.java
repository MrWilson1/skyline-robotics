package org.spartabots.frc2015.action;

import org.spartabots.frc2015.Robot;

import edu.wpi.first.wpilibj.Timer;

/**
 * Action<br/></br>
 * 
 * The init() method is called once to perform the action. The action will continue
 * to be preformed until either the action times out or the action decidess it should
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
	
	protected void run(ActionThread actionThread) {
		timer.start();
		init();
		while (!done && running() && !isTimedOut()) {
			try {
				Thread.sleep(5);
			} catch (InterruptedException e) {
				Thread.currentThread().interrupt();
				e.printStackTrace();
			}
		}
		timer.stop();
		done();
		this.robot = null;
		if (actionThread != null)
			actionThread.actionDone();
	}
	
	public boolean isDone() {
		return done;
	}
	
	private boolean isTimedOut() {
		return timer.get() > timeout;
	}
	
	public void setTimeout(double milliseconds) {
		this.timeout = (milliseconds / 1000.0D);
	}
}
