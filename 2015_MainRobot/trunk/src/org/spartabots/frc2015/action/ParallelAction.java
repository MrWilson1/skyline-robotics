package org.spartabots.frc2015.action;

import java.util.ArrayList;
import java.util.concurrent.CyclicBarrier;

public class ParallelAction extends Action {
	ArrayList<Action> actions = new ArrayList<Action>();
	boolean success = true;
	
	public ParallelAction(Action... actions) {
		for (Action action : actions)
			this.actions.add(action);
	}
	
	public void add(Action action) {
		actions.add(action);
	}
	
	@Override
	public void run(ActionThread thread) {
		init();
		
		if (actions.size() == 0) {
			success = false;
			return;
		}
		if (actions.size() == 1) { // Only one, no need for gate & for lloop
			success = (ActionThread.request(actions.get(0), null) != null);
			return;
		}
		
		final CyclicBarrier gate = new CyclicBarrier(actions.size());
		for (final Action a : actions) {
			if (ActionThread.request(a, gate) == null) {
				gate.reset();
				success = false;
				return;
			}
		}
		this.done = false;
		return;
	}
	
	@Override
	public void init() {
		// Nothing to do here
	}

	@Override
	public boolean running() {
		return false;
	}

	@Override
	public void done() {
		// Nothing to do here
	}

}
