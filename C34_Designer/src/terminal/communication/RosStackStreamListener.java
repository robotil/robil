package terminal.communication;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.Scanner;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import document.BTDesigner;
import document.Document;

import terminal.communication.RosPipe.RosTargets;
import terminal.lineprocessors.LineProcessor;

public class RosStackStreamListener implements Runnable {

	private BTDesigner designer;

	public RosStackStreamListener(BTDesigner designer) {
		this.designer = designer;
	}

	private class StackStreamProcessror implements LineProcessor {

		String buffer = "";

		@Override
		public void onStart() {
		}

		@Override
		public void onNewLine(String line) {
			
			// end of message found: process the message data
			if (line.contains("---")) {
				
				String planID = buffer.substring(
						"data: ExeStack: changed : ".length(), 
						buffer.indexOf(" code=")
					);
				
				Document doc = designer.getDocumentOfRunningPlan(planID);
			
				if (doc == null) {
					return;
				}
				
				doc.cleanRunning();
				doc.setRunning(Utils.getMatchedInstances(buffer, Utils.componentIdRegex));
				
				buffer = "";
				return;
			}

			buffer += line+'\n';
		}

		@Override
		public void onEnd() {
		}

	}

	@Override
	public void run() {
		StackStreamProcessror processor = new StackStreamProcessror();
		RosPipe pipe = new RosPipe(Thread.currentThread(), RosTargets.Topic,
				processor, "echo", RosExecutor.STACK_STREAM);
		
		try {
			pipe.sendAndReceive();
		} catch (IOException ex) {
			System.err.println("Error listening to " + RosExecutor.STACK_STREAM);
		}
	}
}
