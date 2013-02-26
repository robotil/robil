package logger;

import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.concurrent.locks.ReentrantLock;


/**
 * Provides a comfortable way to log debug & trace information 
 * @author blackpc
 *
 */
public class Log {
	
	private static ArrayList<String> _disabledTags = new ArrayList<String>();
	private static ReentrantLock _printLock = new ReentrantLock();
	/**
	 * Prints single line, used by logMessage
	 * @param errorStream Is error stream
	 * @param tag Label for message
	 * @param message Message to print
	 */
	private static void logMessageHelper(boolean errorStream, String tag, String message, boolean debugInformation) {
		StringBuilder lineBuilder = new StringBuilder();
		
		StackTraceElement callingMethod = null;
		StackTraceElement[] stackTraceElements = Thread.currentThread().getStackTrace();
		if (stackTraceElements.length > 0)
			callingMethod = stackTraceElements[stackTraceElements.length - 1];
		
		if (debugInformation) {
			lineBuilder.append("[");
			lineBuilder.append(new SimpleDateFormat("HH:mm:ss").format(new Date()));
			lineBuilder.append("]");
			
			if (callingMethod != null) {
				lineBuilder.append("[");
				
				lineBuilder.append(callingMethod.getClassName());
				lineBuilder.append(".");
				lineBuilder.append(callingMethod.getMethodName());
				
				lineBuilder.append(":");
				lineBuilder.append(callingMethod.getLineNumber());
				
				lineBuilder.append("]");
			}
			
			if (!tag.equals("")) {
				lineBuilder.append("[");
				lineBuilder.append(tag);
				lineBuilder.append("]");
			}
			
			lineBuilder.append(" ");
		}
		
		lineBuilder.append(message);
		
		if (LogManager.isRedirected())
			// Write to redirected stream & standard output			
			LogManager.getCurrentOutputStream().println(lineBuilder.toString());

		if (errorStream)
			LogManager.getStdErrorStream().println(lineBuilder.toString());
		else
			LogManager.getStdOutputStream().println(lineBuilder.toString());
		
		LogManager.getCurrentOutputStream().flush();
		LogManager.getStdOutputStream().flush();
		LogManager.getStdErrorStream().flush();
	}
	
	/**
	 * Prints message to appropriate output stream
	 * @param errorStream Is error stream
	 * @param tag Label for a message
	 * @param message Message to print
	 */
	private static void logMessage(boolean errorStream, String tag, String message) {
		if (_disabledTags.contains(tag))
			return;
		
		_printLock.lock();
		
		String[] lines = message.split("\n");
		boolean multiLine = false;
		
		if (message.contains("\n"))
			multiLine = true;
		
		if (multiLine)
			logMessageHelper(errorStream, tag, "", false);
		
		for (String line : lines) 
			logMessageHelper(errorStream, tag, line, true);
		
		if (multiLine)
			logMessageHelper(errorStream, tag, "", false);
		
		_printLock.unlock();
	}

	/**
	 * Logs debug message
	 * @param message Message to print
	 */
	public static void d(String message) {
		logMessage(false, "DEBUG", message);
	}
	
	/**
	 * Logs error message
	 * @param message Message to print
	 */
	public static void e(String message) {
		logMessage(false, "ERROR", message);
	}
	
	/**
	 * Logs informational message
	 * @param message Message to print
	 */
	public static void i(String message) {
		i("INFO", message);
	}
	
	/**
	 * Logs informational message with custom tag
	 * @param tag Custom label for a message
	 * @param message Message to print
	 */
	public static void i(String tag, String message) {
		logMessage(false, tag, message);
	}
	
	/**
	 * Disables output of specified tags
	 * @param tags Tags list, separated by comma
	 */
	public static void disableTags(String tags) {
		_disabledTags.clear();
		if (tags.trim().isEmpty())
			return;
		
		String[] tagsList = tags.split(",");
		
		for (String tag : tagsList) 
			_disabledTags.add(tag.trim());
	}
}
