package terminal.communication;

/**
 * 
 * @author daner
 *
 */
public class UniversalStackStreamMessageParser extends MessageParser<StackStreamMessage> {
	
	CompactStackStreamMessageParser compact_parser = new CompactStackStreamMessageParser();
	StackStreamMessageParser full_parser = new StackStreamMessageParser();
	
	/**
	 * Parses message from specified string, returns null if parsing fails
	 * @param inputString 
	 * @return Message
	 */
	@Override
	public StackStreamMessage parse(String inputString) {
		if(inputString.indexOf("ExeStack: changed")>=0){
			return full_parser.parse(inputString);
		}else{
			return compact_parser.parse(inputString);
		}
	}

}
