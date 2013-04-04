package terminal.communication;

import java.io.IOException;


import logger.Log;
import terminal.communication.RosPipe.RosTargets;
import terminal.lineprocessors.StopStreamProcessor;
import windows.designer.BTDesigner;

public class RosStopStreamListener implements Runnable {

	
	
	private BTDesigner _designer;

	public RosStopStreamListener(BTDesigner designer) {
		_designer = designer;
	}
	
	@Override
	public void run() {
		StopStreamProcessor processor = new StopStreamProcessor(_designer);
		
		RosPipe pipe = new RosPipe(Thread.currentThread(), RosTargets.Topic,
				processor, "echo", RosExecutor.getStopStreamTopicName());

		try {
			Log.i("STOPSTREAM", "Try to connect to ROS topic: " + RosExecutor.getStopStreamTopicName());
			pipe.sendAndReceive();
		} catch (IOException ex) {
			Log.e("ROS COMMUNICATION CRITICAL ERROR: on listening to " + RosExecutor.getStopStreamTopicName());
			Log.d("WARNING: Designer can not connect to Executer ROS node. You can continue edit and testing plans, but you can't run them."
							+ "\n........ For correct this problem try launch Designer by ROS command : $ roslaunch C34_Designer start.launch");
		}
	}

}
