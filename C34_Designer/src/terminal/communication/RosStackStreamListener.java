package terminal.communication;

import java.io.IOException;

import logger.Log;

import terminal.communication.RosPipe.RosTargets;
import terminal.lineprocessors.LineProcessor;
import document.BTDesigner;
import document.Document;

public class RosStackStreamListener implements Runnable {

	private class StackStreamProcessror implements LineProcessor {

		private String buffer = "";
		private StackStreamMessageParser _messageParser = new StackStreamMessageParser();

		@Override
		public void onEnd() {
			
		}

		@Override
		public void onNewLine(String line) {

			
			// end of message found: process the message data
			if (line.contains("---")) {
				StackStreamMessage message = new StackStreamMessage();
				
				String planID = this.buffer.substring(
						"data: ExeStack: changed : ".length(), this.buffer.indexOf(" code="));

				Document doc = 
						RosStackStreamListener.this.designer.getDocumentOfRunningPlan(planID);

				if (doc == null)
					return;
				
				doc.cleanRunning();
				doc.setRunning(Utils.getMatchedInstances(this.buffer, Utils.componentIdRegex));

				if (_messageParser.tryParse(buffer, message)) {
					doc.onMessageReceive(message);
					Log.d("Message received: ");
					Log.d(message.toString());
				}
				else {
					Log.e("Failed to parse message from ros stack-stream\nBuffer: \n" + buffer + "\n\n===============================\n");
				}
				
				this.buffer = "";
				return;
			}

			this.buffer += line + '\n';
		}

		@Override
		public void onStart() {
		}

	}

	private BTDesigner designer;

	public RosStackStreamListener(BTDesigner designer) {
		this.designer = designer;
	}

	@Override
	public void run() {
		StackStreamProcessror processor = new StackStreamProcessror();
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
