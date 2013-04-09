package terminal.communication;

public abstract class MessageParser<T extends IMessage<T>> {
	
	/**
	 * Tries to parse message, on success, outputMessage will contain the parsed message
	 * @param inputString
	 * @param outputMessage
	 * @return Whether the parsing succeeded
	 */
	public boolean tryParse(String inputString, T outputMessage) {
		T message = parse(inputString);
		
		if (message != null)
			outputMessage.clone(message);
		
		return message != null;
	}
	
	public abstract T parse(String inputMessage);	
}
