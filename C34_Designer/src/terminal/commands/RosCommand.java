package terminal.commands;


import terminal.lineprocessors.LineProcessor;
import terminal.communication.RosPipe;
import terminal.communication.RosPipe.RosTargets;

import terminal.Terminal;

public abstract class RosCommand extends Command {

//	protected Process process; 
	protected RosPipe pipe;
	
	public RosCommand(Terminal terminal, String name) {
		super(terminal, name);
	}

	public void initPipe(RosTargets target, LineProcessor processor, String... args){
		pipe = new RosPipe(thread, target, processor, args);
	}

	
	public boolean isAlreadyRunning(){
		return pipe!=null && pipe.isAlreadyRunning();
	}
	
	public void stop() {
		thread.interrupt();
	}

}
