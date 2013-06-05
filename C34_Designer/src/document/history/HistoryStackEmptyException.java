package document.history;

public class HistoryStackEmptyException extends Exception {

	private static final long serialVersionUID = -7283898384923443513L;
	
	public HistoryStackEmptyException() {
		super("History stack is empty");
	}
	
	public HistoryStackEmptyException(String message) {
		super(message);
	}
	
	public HistoryStackEmptyException(String message, Throwable throwable) {
		super(message, throwable);
	}
	
}
