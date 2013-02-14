package terminal.commands;

import java.util.ArrayList;

import terminal.Terminal;

//------------- PROGRAM --------------------------------
public class History extends Command {

	/**
	 * @param terminal
	 */
	public History(Terminal terminal) {
		super(terminal, "history");
	}

	@Override
	public ArrayList<String> autocomplete(String command) {
		ArrayList<String> ret = new ArrayList<String>();
		ret.add(this.name);
		return ret;
	}

	@Override
	public void execute(String command) {
		int i = this.sys.history.size();
		for (String h : this.sys.history) {
			this.sys.print("" + i + ") " + h + "\n");
			i--;
		}
		return;
	}

	@Override
	public boolean isAutoCompleteAvailable(String command) {
		return isPartOfName(command, this.name);
	}

	@Override
	public void stop() {
	}

}
