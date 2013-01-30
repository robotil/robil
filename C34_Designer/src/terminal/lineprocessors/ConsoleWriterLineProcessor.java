package terminal.lineprocessors;

import terminal.Terminal;

public class ConsoleWriterLineProcessor implements LineProcessor {

	private Terminal terminal;

	public ConsoleWriterLineProcessor(Terminal terminal) {
		this.terminal = terminal;
	}

	@Override
	public void onEnd() {
	}

	@Override
	public void onNewLine(String line) {
		this.terminal.println(line);
	}

	@Override
	public void onStart() {
	}
}
