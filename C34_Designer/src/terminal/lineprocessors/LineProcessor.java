package terminal.lineprocessors;

public interface LineProcessor {
	public void onStart();
	public void onNewLine(String line);
	public void onEnd();
}
