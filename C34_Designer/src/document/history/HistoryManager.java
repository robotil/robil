package document.history;

import java.util.Stack;

import document.Document;

public class HistoryManager {
	
	private Document _document;
	private boolean _ready = false;
	private Snapshot _currentSnapshot;
	private int _sequenceNumber;
	
	private Stack<Snapshot> _undoStack = new Stack<Snapshot>();
	private Stack<Snapshot> _redoStack = new Stack<Snapshot>();
	
	public void init(Document document) throws HistoryManagerNotReadyException {
		
		_document = document;
		_ready = true;
		_sequenceNumber = 0;
		// createSnapshot();
		updateCurrentSnapshot();
		System.out.println("New history session created at ");
		
	}
	
	private void updateCurrentSnapshot() {
		_currentSnapshot = Snapshot.create(this._document, this._sequenceNumber);
		_sequenceNumber++;
	}
	
	public void createSnapshot() throws HistoryManagerNotReadyException {
		if (!isReady())
			throw new HistoryManagerNotReadyException();

		_redoStack.clear();
		_undoStack.push(this._currentSnapshot);
		updateCurrentSnapshot();
		System.out.println("Snapshot saved");
	
	}
	
	public void undo() throws HistoryStackEmptyException, HistoryManagerNotReadyException {
		if (_undoStack.isEmpty())
			throw new HistoryStackEmptyException();
		
		if (!isReady())
			throw new HistoryManagerNotReadyException();
		
		_redoStack.push(this._currentSnapshot);
		
		this._currentSnapshot = _undoStack.pop();
		this._currentSnapshot.activate();

		// _sequenceNumber--;
	}
	
	public void redo() throws Exception {
		if (_redoStack.isEmpty())
			throw new Exception("Redo stack is empty");
		
		if (!isReady())
			throw new HistoryManagerNotReadyException();
		
		_undoStack.push(this._currentSnapshot);
		
		this._currentSnapshot = _redoStack.pop();
		this._currentSnapshot.activate();

		_sequenceNumber++;
	}
	
	public int getUndoCount() {
		return _undoStack.size();
	}
	
	public boolean hasUndo() {
		return _undoStack.size() > 0 && isReady();
	}
	
	public boolean hasRedo() {
		return _redoStack.size() > 0 && isReady();
	}

	public boolean isReady() {
		return _ready;
	}
	
	public void printStacks() {
		
		for (Snapshot snapShot : this._undoStack)
			if (snapShot != null)
				System.out.print(snapShot.toString() + " ");
		
		System.out.print("[" + this._currentSnapshot + "]");
		
		for (Snapshot snapShot : this._redoStack)
			if (snapShot != null)
				System.out.print(snapShot.toString() + " ");
		
		System.out.println();
	}

}
