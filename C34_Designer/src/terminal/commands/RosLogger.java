package terminal.commands;

import java.io.IOException;
import java.util.ArrayList;

import terminal.lineprocessors.BatchLineProcessor;
import terminal.lineprocessors.ConsoleWriterLineProcessor;
import terminal.lineprocessors.LineProcessor;


import terminal.communication.RosPipe.RosTargets;

import terminal.Terminal;

public class RosLogger extends RosCommand {
//	Process process;

	public RosLogger(Terminal terminal) {
		super(terminal, "ros_logger");

	}

	public boolean isAutoCompleteAvailable(String command) {
		return  isPartOfName(command, name)
				|| command.startsWith(name);
	}

	public ArrayList<String> autocomplete(String command) {
		
		String[] args = command.split(" ");
		String topic = "";
		ArrayList<String> ret = new ArrayList<String>();
		
		if (args.length > 1) {
			topic = args[1];
		}
		
		if (name.startsWith(command) ) {
			ret.add(name);
			return ret;
		}
		
		return ret;
	}
	


	public void stop() {
		super.stop();
		//pipe.stop();
	}

	public void execute(String command) {
		if (isAlreadyRunning()) {
			sys.println("ERROR: The previous process is running");
			return;
		}
		String[] args = command.split(" ");
		
		if (thread.isInterrupted()) {
			return;
		}
		
		LineProcessor processor = new ConsoleWriterLineProcessor(this.sys);
		
		if (thread.isInterrupted()) {
			return;
		}

		try {
			initPipe(RosTargets.Service, processor, "call", "/rosout/get_loggers");
			pipe.sendAndReceive();
		} catch (IOException ex) {
			sys.println(ex.getMessage());
		}
		

		return;
	}
	

}
