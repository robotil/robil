package document.history;

import java.util.Stack;

import logger.Log;

import document.Document;

/**
 * Stores and manages history of document, enabling undoing or redoing changes made to a document
 * @author blackpc
 *
 */
public class HistoryManager {
	
	private Document _document;
	private boolean _ready = false;
	private Snapshot _currentSnapshot;
	private int _sequenceNumber;
	
	private Stack<Snapshot> _undoStack = new Stack<Snapshot>();
	private Stack<Snapshot> _redoStack = new Stack<Snapshot>();
	
	/**
	 * Makes snapshot of current state
	 */
	private void updateCurrentSnapshot() {
		_currentSnapshot = Snapshot.create(_document, _sequenceNumber);
		_sequenceNumber++;
	}
	
	/**
	 * Initializes the manager to allow undo & redo actions
	 * Must be called before any undo/redo/createSnapshot actions 
	 * @param document Target document
	 * @throws HistoryManagerNotReadyException
	 */
	public void init(Document document) throws HistoryManagerNotReadyException {
		_document = document;
		_ready = true;
		_sequenceNumber = 0;
		// createSnapshot();
		updateCurrentSnapshot();
	}
	
	/**
	 * Creates new snapshot, must be called before the change of document's elements
	 * @throws HistoryManagerNotReadyException
	 */
	public void createSnapshot() throws HistoryManagerNotReadyException {
		if (!isReady())
			throw new HistoryManagerNotReadyException();

		_redoStack.clear();
		_undoStack.push(_currentSnapshot);
		updateCurrentSnapshot();
	}
	
	/**
	 * Undo last change
	 * @throws HistoryStackEmptyException
	 * @throws HistoryManagerNotReadyException
	 */
	public void undo() throws HistoryStackEmptyException, HistoryManagerNotReadyException {
		if (_undoStack.isEmpty())
			throw new HistoryStackEmptyException();
		
		if (!isReady())
			throw new HistoryManagerNotReadyException();
		
		_redoStack.push(_currentSnapshot);
		
		_currentSnapshot = _undoStack.pop();
		_currentSnapshot.activate();
	}
	
	/**
	 * Redo last undo action
	 * @throws HistoryStackEmptyException
	 * @throws HistoryManagerNotReadyException
	 */
	public void redo() throws Exception {
		if (_redoStack.isEmpty())
			throw new HistoryStackEmptyException();
		
		if (!isReady())
			throw new HistoryManagerNotReadyException();
		
		_undoStack.push(_currentSnapshot);
		
		_currentSnapshot = _redoStack.pop();
		_currentSnapshot.activate();

		_sequenceNumber++;
	}
	
	/**
	 * Returns number of available undo steps
	 */
	public int getUndoCount() {
		return _undoStack.size();
	}
	
	/**
	 * Is undo action available 
	 * @return
	 */
	public boolean hasUndo() {
		return _undoStack.size() > 0 && isReady();
	}
	
	/**
	 * Is redo action available
	 * @return
	 */
	public boolean hasRedo() {
		return _redoStack.size() > 0 && isReady();
	}

	/**
	 * Is manager redo for work
	 * @return
	 */
	public boolean isReady() {
		return _ready;
	}
	
	/**
	 * Prints information about current state, undo & redo stacks to standard output
	 */
	public void printStacks() {
		StringBuilder output = new StringBuilder();
		
		for (Snapshot snapShot : this._undoStack)
			if (snapShot != null)
				output.append(snapShot.toString() + " ");
		
		output.append("[" + this._currentSnapshot + "]");
		
		for (Snapshot snapShot : this._redoStack)
			if (snapShot != null)
				output.append(snapShot.toString() + " ");
		
		Log.e(output.toString());
	}

}
