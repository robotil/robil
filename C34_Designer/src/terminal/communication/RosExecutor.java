package terminal.communication;

import java.io.IOException;
import java.util.ArrayList;
import java.util.concurrent.*;

import document.BTDesigner;

import terminal.commands.ServiceCaller;
import terminal.communication.RosPipe.RosTargets;
import terminal.lineprocessors.DummyLineProcessor;

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
		workerThreads = new ArrayList<Thread>();
		workerThreads.add(new Thread(new RosStackStreamListener(designer),
				"RosStackListener"));
		workerThreads.add(new Thread(new RosStopStreamListener(),
				"RosStopListener"));

		for (Thread t : workerThreads) {
			t.start();
		}
	}

	public void shutDownAll() {
		for (Thread t : workerThreads) {
			t.interrupt();
		}
	}

	public void runBehaviorTree(String btName, String plan) {
		ServiceCaller caller = new ServiceCaller();
		caller.callService(RUN, btName, plan);
	}

	public void resumeBehaviorTree(String btName) {
		ServiceCaller caller = new ServiceCaller();
		caller.callService(RESUME, btName);
	}

	public void stopBehaviorTree(String btName) {
		ServiceCaller caller = new ServiceCaller();
		caller.callService(STOP, btName);
	}
	
	public void stepBehaviorTree(String btName) {
		ServiceCaller caller = new ServiceCaller();
		caller.callService(STEP, btName);
	}
	
	public void pauseBehaviorTree(String btName) {
		ServiceCaller caller = new ServiceCaller();
		caller.callService(PAUSE, btName);
	}
	
}
