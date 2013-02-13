package terminal.commands;

import java.util.ArrayList;

import terminal.Terminal;

//------------- PROGRAM --------------------------------
public class Test extends Command {

	/**
	 * @param terminal
	 */
	public Test(Terminal terminal) {
		super(terminal, "test");
	}

	@Override
	public ArrayList<String> autocomplete(String command) {
		ArrayList<String> ret = new ArrayList<String>();
		ret.add(this.name);
		return ret;
	}

	@Override
	public void execute(String command) {
		this.sys.println("TEST PROGRAM");
		return;
	}

	@Override
	public boolean isAutoCompleteAvailable(String command) {
		return isPartOfName(command, this.name);
	}

	@Override
	public boolean isRelatedTo(String command) {
		return isNameOfProgram(command, this.name);
	}

	@Override
	public void stop() {
	}

}