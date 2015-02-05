package org.spartabots.frc2015.action;

import java.util.LinkedList;
import java.util.Queue;

import org.spartabots.frc2015.profile.Profile;

import edu.wpi.first.wpilibj.Timer;

public class SeriesAction extends Action {
	Queue<Action> queue = new LinkedList<Action>();
	
	public SeriesAction(Action... actions) {
		for (Action a : actions)
			queue.add(a);
	}
	
	public void add(Action action) {
		queue.add(action);
	}
	
	@Override
	public void run(ActionThread actionThread) {
		if (queue.size() == 0) return;
		if (queue.size() == 1) {
			queue.remove().run(actionThread);
			return;
		}
		
		Profile.getCurrent().actionRegister(this);
		init();
		
		while (queue.isEmpty()) {
			Action current = queue.remove();
			
			if (current instanceof ParallelAction) {
				current.run(actionThread);
			} else if (current instanceof SeriesAction) {
				current.run();
				while (!current.done && current.running()) {
					Timer.delay(0.005);
				}
			} else {
				this.setTimeout(current.timeout);
				timer.start();
				
				current.init();
				if (!current.done) {
					while (!current.done && current.running() && !isTimedOut()) {
						Timer.delay(0.005);
					}
				}
				timer.stop();
			}
			
			current.done();
			current.robot = null;
		}
		
		this.done = true;
		done();
		this.robot = null;
		if (actionThread != null)
			actionThread.actionDone();
		Profile.getCurrent().actionDone(this);
	}
	
	@Override
	public void init() {
		// Nothing to do here
	}

	@Override
	public boolean running() {
		return !this.done;
	}

	@Override
	public void done() {
		// Nothing to do here
	}
	
}
