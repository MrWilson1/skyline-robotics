package org.spartabots.frc2015.action;

import java.util.LinkedList;
import java.util.Queue;

public class SeriesAction extends Action {
	Queue<Action> queue = new LinkedList<Action>();
	
	public SeriesAction(Action... actions) {
		this.canTimeOut = false;
		for (Action a : actions)
			queue.add(a);
	}
	
	/**
	 * Add action to this series. Should not be used after start.
	 * 
	 * @param action
	 */
	public void enqueue(Action action) {
		queue.add(action);
	}

	/**
	 * Add action to this series. Should not be used after start.
	 * 
	 * @param action
	 */
	public void enqueueToProfile(Action action) {
		action.enqueue2 = true;
		queue.add(action);
	}
	
	@Override
	public void init() {
		
	}

	@Override
	public final boolean runPeriodic() {
		if (!queue.isEmpty()) {
			Action current = queue.peek();
			if (!current.runPeriodic())
				queue.remove();
			return true;
		} else {
			return false;
		}
	}

	@Override
	public void done() {
		queue.clear();
	}
	
}