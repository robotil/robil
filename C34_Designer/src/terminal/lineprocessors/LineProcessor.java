package terminal.lineprocessors;

public interface LineProcessor {
	public void onEnd();

	public void onNewLine(String line);

	public void onStart();
}
