package org.spartabots.frc2015.action;

import java.util.LinkedList;
import java.util.Queue;

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
		
		init();
		
		while (queue.isEmpty() && !done) {
			Action current = queue.remove();
			
			if (current instanceof ParallelAction || current instanceof SeriesAction) {
				current.run();
			} else {
				this.setTimeout(current.timeout);
				timer.start();
				
				current.init();
				if (!current.done) {
					while (!this.done && !current.done && current.running() && !isTimedOut()) {
						Timer.delay(0.005);
					}
				}
				timer.stop();
			}
			
			current.done();
		}
		
		this.done = true;
		done();
		if (actionThread != null)
			actionThread.actionDone();
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
