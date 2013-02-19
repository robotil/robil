package logger;

import java.text.SimpleDateFormat;
import java.util.Date;

public class Log {
	
	private static void logMessage(boolean errorStream, String tag, String message) {
		StringBuilder lineBuilder = new StringBuilder();
		
		StackTraceElement callingMethod = null;
		StackTraceElement[] stackTraceElements = Thread.currentThread().getStackTrace();
		if (stackTraceElements.length > 0)
			callingMethod = stackTraceElements[stackTraceElements.length - 1];
		
		lineBuilder.append("[");
		lineBuilder.append(new SimpleDateFormat("HH:mm:ss").format(new Date()));
		lineBuilder.append("]");
		
		if (callingMethod != null) {
			lineBuilder.append("[");
			
			lineBuilder.append(callingMethod.getClassName());
			lineBuilder.append(".");
			lineBuilder.append(callingMethod.getMethodName());
			
			lineBuilder.append("]");
		}
		
		if (!tag.equals("")) {
			lineBuilder.append("[");
			lineBuilder.append(tag);
			lineBuilder.append("]");
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

	public static void d(String message) {
		logMessage(false, "DEBUG", message);
	}
	
	public static void e(String message) {
		logMessage(false, "ERROR", message);
	}
	
	public static void i(String message) {
		i("INFO", message);
	}
	
	public static void i(String tag, String message) {
		logMessage(false, tag, message);
	}
}
