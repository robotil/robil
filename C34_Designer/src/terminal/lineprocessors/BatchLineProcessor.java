package terminal.lineprocessors;

import java.util.ArrayList;

public class BatchLineProcessor implements LineProcessor {

	private ArrayList<String> lines = new ArrayList<String>();

	public BatchLineProcessor() {
	}

	@Override
	public void onStart() {
	}

	@Override
	public void onNewLine(String line) {
		lines.add(new String(line));
	}

	@Override
	public void onEnd() {
	}

	public ArrayList<String> getLines() {
		return new ArrayList<String>(lines);
	}
	
	public ArrayList<String> getLines(String filter, String pref, String suf) {
		ArrayList<String> r = new ArrayList<String>();
		for (String w : lines)
			if ((pref + w + suf).startsWith(filter))
				r.add(pref + w + suf);
		return r;
	}

	public void clear() {
		lines.clear();
	}
}
