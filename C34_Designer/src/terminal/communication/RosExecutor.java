package terminal.communication;

import java.util.ArrayList;

import logger.Log;

import terminal.commands.ServiceCaller;
import document.BTDesigner;

public class RosExecutor {

	public static final String EXECUTER = "/executer/";
	public static final String RUN = EXECUTER + "run";
	public static final String STOP = EXECUTER + "stop";
	public static final String RESUME = EXECUTER + "resume";
	public static final String STEP = EXECUTER + "step";
	public static final String PAUSE = EXECUTER + "pause";
	public static final String STACK_STREAM = EXECUTER + "stack_stream";
	public static final String STOP_STREAM = EXECUTER + "stop_stream";

	private ArrayList<Thread> workerThreads;

	public RosExecutor(BTDesigner designer) {

		// start ROS listeners
		this.workerThreads = new ArrayList<Thread>();
		this.workerThreads.add(new Thread(new RosStackStreamListener(designer),
				"RosStackListener"));
		this.workerThreads.add(new Thread(new RosStopStreamListener(designer),
				"RosStopListener"));

		for (Thread t : this.workerThreads)
			t.start();
	}

	public void pauseBehaviorTree(String btName) {
		ServiceCaller caller = new ServiceCaller();
		Log.i("ROSEXECUTOR", "Pause " + btName);
		caller.callService(PAUSE, btName);
	}

	public void resumeBehaviorTree(String btName) {
		Log.i("ROSEXECUTOR", "Resume " + btName);
		ServiceCaller caller = new ServiceCaller();
		caller.callService(RESUME, btName);
	}

	public void runBehaviorTree(String btName, String plan) {
		ServiceCaller caller = new ServiceCaller();
		Log.i("ROSEXECUTOR", "Run " + btName);
		caller.callService(RUN, btName, plan);
	}

	public void shutDownAll() {
		for (Thread t : this.workerThreads) {
			t.interrupt();
		}
	}

	public void stepBehaviorTree(String btName) {
		ServiceCaller caller = new ServiceCaller();
		Log.i("ROSEXECUTOR", "Step " + btName);
		caller.callService(STEP, btName);
	}

	public void stopBehaviorTree(String btName) {
		ServiceCaller caller = new ServiceCaller();
		Log.i("ROSEXECUTOR", "Stop " + btName);
		caller.callService(STOP, btName);
	}

}
