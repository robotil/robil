package terminal;

import terminal.commands.History;
import terminal.commands.RosLogger;
import terminal.commands.ServiceCaller;
import terminal.commands.Test;
import terminal.commands.TopicListener;

public class ProgramRegistrator {
	static public void register(Terminal sys) {
		Terminal.registerProgram(new History(sys));
		Terminal.registerProgram(new Test(sys));
		Terminal.registerProgram(new TopicListener(sys));
		Terminal.registerProgram(new RosLogger(sys));
		Terminal.registerProgram(new ServiceCaller(sys));
	}
}
