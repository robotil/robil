package terminal.lineprocessors;

import java.util.ArrayList;

public class ArrayLineProcessor implements LineProcessor {

	private ArrayList<LineProcessor> processors = new ArrayList<LineProcessor>();
	
	public ArrayLineProcessor() {}

	public void add(LineProcessor processor) {
		processors.add(processor);
	}
	
	@Override
	public void onStart() {
		for (LineProcessor processor : processors) {
			processor.onStart();
		}
	}

	@Override
	public void onNewLine(String line) {
		for (LineProcessor processor : processors) {
			processor.onNewLine(line);
		}
	}

	@Override
	public void onEnd() {
		for (LineProcessor processor : processors) {
			processor.onEnd();
		}
	}
}
