package terminal;

import terminal.commands.*;
import terminal.Terminal;

public class ProgramRegistrator {
	static public void register(Terminal sys){
		Terminal.registerProgram(new History(sys));
		Terminal.registerProgram(new Test(sys));
		Terminal.registerProgram(new TopicListener(sys));
		Terminal.registerProgram(new RosLogger(sys));
		Terminal.registerProgram(new ServiceCaller(sys));
	}
}
