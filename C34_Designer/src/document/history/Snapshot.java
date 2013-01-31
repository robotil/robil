package document.history;

import java.io.File;
import java.util.Date;

import document.Document;

class Snapshot {
	
	private String _fileName;
	private Date _date;
	private boolean _immutable = true;
	
	public Snapshot(String fileName) {
		this._date = new Date();
		this._fileName = fileName;
	}
	
	public static Snapshot create(Document document, String fileName) {
		Snapshot snapshot = new Snapshot(fileName);
		snapshot._immutable = false;
		return document.compile(fileName, false, false) ? snapshot : null;
	}
	
	public String getFilename() {
		return _fileName;
	}
	
	public Date getDate() {
		return _date;
	}
	
	public void delete() {
		if (!_immutable)
			new File(_fileName).delete();
	}
	
	@Override
	public String toString() {
		return _fileName;
	}
}
