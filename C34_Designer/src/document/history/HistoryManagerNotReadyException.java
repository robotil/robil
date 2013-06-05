package document.history;

public class HistoryManagerNotReadyException extends Exception {

	private static final long serialVersionUID = -3820591600130974539L;

	public HistoryManagerNotReadyException() {
		super("History manager must be initialized with init() call");
	}
	
	/**
	 * Constructs with the given throwable
	 * @param throwable The throwable to throw
	 */
	public HistoryManagerNotReadyException(Throwable throwable) {
		super(throwable);
	}

	/**
	 * Constructs with the given message
	 * @param message The message of the exception
	 */
	public HistoryManagerNotReadyException(String message) {
		super(message);
	}

	/**
	 * Constructs with the given message and the original throwable cause
	 * @param message The message of the exception
	 * @param throwable The original throwable
	 */
	public HistoryManagerNotReadyException(String message, Throwable throwable) {
		super(message, throwable);
	}
}
