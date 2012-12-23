package terminal.commands;

import java.util.ArrayList;

import terminal.Terminal;

/**
 * Represents an abstract command which is executed in terminal (command pattern)
 * http://en.wikipedia.org/wiki/Command_pattern  
 * @author matan
 *
 */
public abstract class Command {

	protected String name;		// command name
	protected Terminal sys;		// reference to the terminal 
	public Thread thread;		// thread which executes command

	/**
	 * Creates a new command
	 * @param terminal reference to a terminal which executes the command
	 * @param name command name
	 */
	public Command(Terminal terminal, String name) {
		sys = terminal;
		this.name = name;
	}

	public Command(String name) {
		this(null, name);
	}

	public boolean isNameOfProgram(String command, String name) {
		if (command.startsWith(name)) {
			if (command.length() == name.length())
				return true;
			return command.startsWith(name + " ");
		}
		return false;
	}

	public abstract boolean isAutoCompleteAvailable(String command);

	public boolean isPartOfName(String command, String name) {
		return name.startsWith(command);
	}

	public boolean isRelatedTo(String command) {
		return isNameOfProgram(command, name);
	}

	/**
	 * Executes command according to arguments
	 * @param command command arguments
	 * @return
	 */
	public abstract void execute(String command);

	public abstract ArrayList<String> autocomplete(String command);
	
	public void init(Terminal terminal) {
		sys = terminal;
	}

	public abstract void stop();

}