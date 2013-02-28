package terminal.communication;

import java.util.regex.Matcher;
import java.util.regex.Pattern;

import logger.Log;
import logger.LogManager;

/**
 * 
 * @author blackpc
 *
 */
public class StackStreamMessageParser {
	
	private static final String SUCCESS_OR_FAILURE_PATTERN =
			"(?:.*?(FAILURE|SUCCESS)\\((\\-?\\d+)\\)\\:?([^\\$]*))?";
	
	private static final String MESSAGE_TASK_ID_PATTERN = 
			"(?:\\[id=([\\w\\-\\_]+)\\])?" + SUCCESS_OR_FAILURE_PATTERN;
	
	private static final String MESSAGE_FORMAT_PATTERN =   
			"data: ExeStack: changed : (\\w+) code=(\\d), node=(\\w+)\\((.*?)(?:\\)\\(|\\)\\s|\\)$)\\s?" + MESSAGE_TASK_ID_PATTERN; // [id=PP_ID]($Task(PathPlanning) [id=PP_ID]:FAILURE(1000)$)";
	
	private final Pattern _messageRegexPattern = Pattern.compile(MESSAGE_FORMAT_PATTERN);
	
	/**
	 * Tries to parse message, on success, outMessage will contain the parsed message
	 * @param inputString
	 * @param outMessage
	 * @return Whether the parsing succeeded
	 */
	public boolean tryParse(String inputString, StackStreamMessage outMessage) {
		StackStreamMessage message = parse(inputString);
		
		if (message != null)
			outMessage.clone(message);
		
		return message != null;
	}
	
	/**
	 * Parses message from specified string, returns null if parsing fails
	 * @param inputString 
	 * @return Message
	 */
	public StackStreamMessage parse(String inputString) {
		StackStreamMessage message = new StackStreamMessage();
		
		Matcher m = _messageRegexPattern.matcher(inputString);
		
		if (m.find()) {
			try {
				message.setPlanLabel(m.group(1));
				message.setChangeType(Integer.parseInt(m.group(2)));
				message.setTaskName(m.group(3));
				message.setTaskParameters(m.group(4));
				message.setTaskId(m.group(5));
				message.setTaskFinishReason(m.group(6));
				
				if (m.group(7) != null && m.group(7).length() > 0)
					message.setTaskResultCode(Integer.parseInt(m.group(7)));
				
				message.setTaskResultDescription(m.group(8));
			} catch (Exception ex) {
				message = null;
			}
		} else
			message = null; // No match
		
		return message;
	}
	
	public static void main(String[] args) {
		LogManager.redirectStandardAndErrorOutput("test_stdoutput.txt");
		Log.d("Test message");
		
		Log.i("Test message");
	}
}
