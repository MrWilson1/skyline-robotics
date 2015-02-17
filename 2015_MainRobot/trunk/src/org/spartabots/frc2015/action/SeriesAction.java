package org.spartabots.frc2015.action;

import java.util.LinkedList;
import java.util.Queue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public abstract class SeriesAction extends Action {
	protected Queue<Action> queue = new LinkedList<Action>();
	
	public SeriesAction(Action... actions) {
		this.setTimeout(20000);
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
	public final boolean runPeriodic() {
		if (!queue.isEmpty()) {
			Action current = queue.peek();
			SmartDashboard.putString("Series", current.toString());
			if (current instanceof SeriesAssert) {
				SeriesAssert seriesAssert = (SeriesAssert) current;
				if (!seriesAssert.check()) {
					return false;
				} else {
					queue.remove();
					return true;
				}
			}
			
			if (current.enqueue2) {
				robot.profile.add(current);
				queue.remove();
			} else if (!current.running()) {
				queue.remove();
			}
			return true;
		} else {
			return false;
		}
	}

	@Override
	public void done() {
		if (!queue.isEmpty()) {
			while (!queue.isEmpty()) {
				Action a = queue.remove();
				if (!a.isDone()) {
					a.cancel();
				}
			}
		}
	}
	
}
