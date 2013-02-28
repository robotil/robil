package terminal.commands;

import java.io.IOException;
import java.util.ArrayList;

import terminal.Terminal;
import terminal.communication.RosPipe.RosTargets;
import terminal.lineprocessors.ConsoleWriterLineProcessor;
import terminal.lineprocessors.LineProcessor;

public class RosLogger extends RosCommand {
	// Process process;

	public RosLogger(Terminal terminal) {
		super(terminal, "ros_logger");

	}

	@Override
	public ArrayList<String> autocomplete(String command) {

		String[] args = command.split(" ");
		@SuppressWarnings("unused")
		String topic = "";
		ArrayList<String> ret = new ArrayList<String>();

		if (args.length > 1) {
			topic = args[1];
		}

		if (this.name.startsWith(command)) {
			ret.add(this.name);
			return ret;
		}

		return ret;
	}

	@Override
	public void execute(String command) {
		if (isAlreadyRunning()) {
			this.sys.println("ERROR: The previous process is running");
			return;
		}
		@SuppressWarnings("unused")
		String[] args = command.split(" ");

		if (this.thread.isInterrupted()) {
			return;
		}

		LineProcessor processor = new ConsoleWriterLineProcessor(this.sys);

		if (this.thread.isInterrupted()) {
			return;
		}

		try {
			initPipe(RosTargets.Service, processor, "call",
					"/rosout/get_loggers");
			this.pipe.sendAndReceive();
		} catch (IOException ex) {
			this.sys.println(ex.getMessage());
		}

		return;
	}

	@Override
	public boolean isAutoCompleteAvailable(String command) {
		return isPartOfName(command, this.name)
				|| command.startsWith(this.name);
	}

	@Override
	public void stop() {
		super.stop();
		// pipe.stop();
	}

}
