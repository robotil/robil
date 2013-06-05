package terminal.communication;

import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Compact stack stream message parser
 * 
 * 
 * Compact stack stream format in BNF (Backus–Naur Form)
 * =====================================================
 * 
 *  <change-type>         ::= 0 | 1			; 0 - Started, 1 - Finished
 *	<task-id>             ::= UID
 *	<plan-id>             ::= UID
 *	<return-value>        ::= signed integer
 *	<return-description>  ::= string
 *	
 *	<stack-message> ::= <change-type> ":" <plan-id> ":" <task-id> [ ":" <return-value> [ ":" <return-description> ] ]
 * 
 * @author blackpc
 *
 */
public class CompactStackStreamMessageParser extends MessageParser<StackStreamMessage> {

	private static final String MESSAGE_PATTERN =
			"(\\d)\\:([\\w\\-\\_]+)\\:([\\w\\-\\_]+)(?:\\:(-?\\d+)(?:\\:(.*))?)?";
			// "(.*):(.*):(.*):(.*):(.*)$";

	private final Pattern _messageRegexPattern = Pattern.compile(MESSAGE_PATTERN);
	
	@Override
	public StackStreamMessage parse(String inputMessage) {
		StackStreamMessage message = new StackStreamMessage();
		
		Matcher m = _messageRegexPattern.matcher(inputMessage);
		
		if (m.find()) {
			try {
				message.setChangeType(Integer.parseInt(m.group(1)));
				message.setPlanLabel(m.group(2));
				message.setTaskId(m.group(3));
				
				if (m.group(4) != null)
					message.setTaskResultCode(Integer.parseInt(m.group(4)));
				
				if (m.group(5) != null)
					message.setTaskResultDescription(m.group(5));
					
			} catch (Exception ex) {
				message = null;
			}
		} else
			message = null; // No match
		
		return message;
	}

}
