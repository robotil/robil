package terminal.commands;

import terminal.Terminal;
import terminal.communication.RosPipe;
import terminal.communication.RosPipe.RosTargets;
import terminal.lineprocessors.LineProcessor;

public abstract class RosCommand extends Command {

	// protected Process process;
	protected RosPipe pipe;

	public RosCommand(Terminal terminal, String name) {
		super(terminal, name);
	}

	public void initPipe(RosTargets target, LineProcessor processor,
			String... args) {
		this.pipe = new RosPipe(this.thread, target, processor, args);
	}

	public boolean isAlreadyRunning() {
		return this.pipe != null && this.pipe.isAlreadyRunning();
	}

	@Override
	public void stop() {
		this.thread.interrupt();
	}

}
