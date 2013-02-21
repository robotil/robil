package terminal.communication;

import java.io.IOException;

import logger.Log;

import terminal.communication.RosPipe.RosTargets;
import terminal.lineprocessors.StackStreamProcessor;
import document.BTDesigner;

public class RosStackStreamListener implements Runnable {

	private BTDesigner _designer;

	public RosStackStreamListener(BTDesigner designer) {
		this._designer = designer;
	}

	@Override
	public void run() {
		StackStreamProcessor processor = new StackStreamProcessor(_designer);
		RosPipe pipe = new RosPipe(Thread.currentThread(), RosTargets.Topic,
				processor, "echo", RosExecutor.STACK_STREAM);

		try {
			Log.d("Try to connect to ROS topic: "+RosExecutor.STACK_STREAM);
			pipe.sendAndReceive();
		} catch (IOException ex) {
			Log.e("ROS COMMUNICATION CRITICAL ERROR: on listening to "
							+ RosExecutor.STACK_STREAM);
			Log.d("WARNING: Designer can not connect to Executer ROS node. You can continue edit and testing plans, but you can't run them."
							+ "\n........ For correct this problem try launch Designer by ROS command : $ roslaunch C34_Designer start.launch");
		}
	}
}
