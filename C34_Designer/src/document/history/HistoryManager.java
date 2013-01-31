package document.history;

import java.io.File;
import java.util.Stack;

import document.Document;

public class HistoryManager {
	
	private final String SESSION_DIRECTORY_PATTERN 	= "{HISTORY_DIRECTORY}/{SESSION_ID}";
	private final String SNAPSHOT_FILENAME_PATTERN 	= "{SEQUENCE_NUMBER}_{SESSION_ID}";
	
	private Document _document;
	private File _historyDirectory;
	private File _sessionDirectory;
	private int _sequenceNumber = 0;
	private boolean _ready = false;
	
	private Stack<Snapshot> _undoStack = new Stack<Snapshot>();
	private Stack<Snapshot> _redoStack = new Stack<Snapshot>();
	private String _sessionId;
	
	private String getSessionDirectory(String historyDirectory, String sessionId) {
		return SESSION_DIRECTORY_PATTERN
				.replace("{HISTORY_DIRECTORY}", historyDirectory)
				.replace("{SESSION_ID}", sessionId);
	}
	
	private boolean isSessionExists(File sessionDirectory) {
		if (!sessionDirectory.exists())
			return false;
		
		return sessionDirectory.listFiles().length > 0;
	}
	
	private void restoreSession(File sessionDirectory) {
		
	}
	
	private void createSessionDirectory(File sessionDirectory) {
		if (!_historyDirectory.exists()) 
			_historyDirectory.mkdir();
		
		if (sessionDirectory.exists() && sessionDirectory.isDirectory())
			return;
		
		boolean directoryCreated = sessionDirectory.mkdir();
		
		if (!directoryCreated) {
			
		}
	}
	
	private void removeSessionDirectory() {
		if (!_sessionDirectory.exists())
			return;
		
		for (File file : _sessionDirectory.listFiles()) 
			file.delete();
		
		_sessionDirectory.delete();
	}
	
	private String getCurrentFileName() {
		return SNAPSHOT_FILENAME_PATTERN
				.replace("{SEQUENCE_NUMBER}", Integer.toString(_sequenceNumber))
				.replace("{SESSION_ID}", _sessionId);
	}
	
	public void init(Document document, String historyDirectory, String sessionId) throws HistoryManagerNotReadyException {
		
		_document = document;
		_historyDirectory = new File(historyDirectory);
		_sessionDirectory = new File(getSessionDirectory(historyDirectory, sessionId));
		_sessionId = sessionId;
		
		if (isSessionExists(_sessionDirectory)) {
			restoreSession(_sessionDirectory);
			_ready = true;
			// createSnapshot();
			System.out.println("Session restored from " + _sessionDirectory.getAbsolutePath());
		}
		else {
			createSessionDirectory(_sessionDirectory);
			_ready = true;
			createSnapshot();
			System.out.println("New history session created at " + _sessionDirectory.getAbsolutePath());
		}
	}
	
	public void finalize() {
		removeSessionDirectory();
	}
	
	public void createSnapshot() throws HistoryManagerNotReadyException {
		if (!isReady())
			throw new HistoryManagerNotReadyException();
		
		String absolutePath = new File(_sessionDirectory.getAbsolutePath(), getCurrentFileName()).getPath();
		Snapshot snapshot = Snapshot.create(_document, absolutePath);
		
		if (snapshot != null) {
			_undoStack.push(snapshot);
			// _currentSnapshot = snapshot;
			_redoStack.clear();
			_sequenceNumber++;
			System.out.println("Snapshot saved at " + absolutePath);
		} else {
			System.out.println("Snapshot not created (Document compilation failed) at " + absolutePath);
		}
	}
	
	public void undo() throws HistoryStackEmptyException, HistoryManagerNotReadyException {
		if (_undoStack.isEmpty())
			throw new HistoryStackEmptyException();
		
		if (!isReady())
			throw new HistoryManagerNotReadyException();
		
		Snapshot currentSnapshot = _undoStack.pop();
		_redoStack.push(currentSnapshot);
		_document.loadPlan(_undoStack.peek().getFilename(), false);
		// _currentSnapshot = latestSnapshot;
		_sequenceNumber--;
	}
	
	public void redo() throws Exception {
		if (_redoStack.isEmpty())
			throw new Exception("Redo stack is empty");
		
		if (!isReady())
			throw new HistoryManagerNotReadyException();
		
		Snapshot snapshot = _redoStack.pop();
		_document.loadPlan(snapshot.getFilename(), false);
		_undoStack.push(snapshot);
		// _currentSnapshot = snapshot;
		_sequenceNumber++;
	}
	
	public boolean hasUndo() {
		return _undoStack.size() > 1 && isReady();
	}
	
	public boolean hasRedo() {
		return _redoStack.size() > 0 && isReady();
	}

	public boolean isReady() {
		return _ready;
	}
	

}
