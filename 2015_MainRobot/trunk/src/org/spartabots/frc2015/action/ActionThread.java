package org.spartabots.frc2015.action;

import java.util.Collections;
import java.util.HashSet;
import java.util.Iterator;
import java.util.Set;
import java.util.concurrent.BrokenBarrierException;
import java.util.concurrent.CyclicBarrier;

class ActionThread extends Thread {
	private static boolean running = true;
	private static final int THREAD_LIMIT = 50;
	private static final int INITIAL_SIZE = 15;
	private static Set<ActionThread> threads = Collections.synchronizedSet(new HashSet<ActionThread>());
	
	public Action myAction;
	private boolean actionActive = false;
	private CyclicBarrier gate;
	
	static {
		for (int i = 0; i < INITIAL_SIZE; i++) {
			threads.add(new ActionThread());
		}
	}

	public static ActionThread request(Action action) {
		return request(action, null);
	}
	
	public static ActionThread request(Action action, CyclicBarrier gate) {
		synchronized (threads) {
			Iterator<ActionThread> itr = threads.iterator();
			while (itr.hasNext()) { // find inactive thread to ue
				ActionThread at = itr.next();
				if (!at.actionActive) {
					at.gate = gate;
					at.myAction = action;
					return at;
				}
			}
			
			if (threads.size() < THREAD_LIMIT) { // create new thread if thread limit is not passed
				ActionThread at = new ActionThread();
				threads.add(at);
				at.gate = gate;
				at.myAction = action;
				return at;
			}
		}
		return null; // will return null if thread limit passed
	}
	
	public static void shutdown() {
		running = false;
	}
	
	@Override
	public void run() {
		while (running) {
			if (!actionActive && myAction != null) {
				actionActive = true;
				
				if (gate != null) {
					try {
						gate.await();
						myAction.run(this);
					} catch (InterruptedException | BrokenBarrierException e) {
						actionDone();
					}
				} else {
					myAction.run(this);
				}
			}
		}
		actionDone();
	}
	
	public void actionDone() {
		gate = null;
		myAction = null;
		actionActive = false;
	}
}
