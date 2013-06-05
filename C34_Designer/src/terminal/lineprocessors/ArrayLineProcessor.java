package terminal.lineprocessors;

import java.util.ArrayList;

public class ArrayLineProcessor implements LineProcessor {

	private ArrayList<LineProcessor> processors = new ArrayList<LineProcessor>();

	public ArrayLineProcessor() {
	}

	public void add(LineProcessor processor) {
		this.processors.add(processor);
	}

	@Override
	public void onEnd() {
		for (LineProcessor processor : this.processors) {
			processor.onEnd();
		}
	}

	@Override
	public void onNewLine(String line) {
		for (LineProcessor processor : this.processors) {
			processor.onNewLine(line);
		}
	}

	@Override
	public void onStart() {
		for (LineProcessor processor : this.processors) {
			processor.onStart();
		}
	}
}
