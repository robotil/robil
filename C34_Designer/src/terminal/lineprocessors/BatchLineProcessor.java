package terminal.lineprocessors;

import java.util.ArrayList;

public class BatchLineProcessor implements LineProcessor {

	private ArrayList<String> lines = new ArrayList<String>();

	public BatchLineProcessor() {
	}

	public void clear() {
		this.lines.clear();
	}

	public ArrayList<String> getLines() {
		return new ArrayList<String>(this.lines);
	}

	public ArrayList<String> getLines(String filter, String pref, String suf) {
		ArrayList<String> r = new ArrayList<String>();
		for (String w : this.lines)
			if ((pref + w + suf).startsWith(filter))
				r.add(pref + w + suf);
		return r;
	}

	@Override
	public void onEnd() {
	}

	@Override
	public void onNewLine(String line) {
		this.lines.add(new String(line));
	}

	@Override
	public void onStart() {
	}
}
