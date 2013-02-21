package terminal.lineprocessors;

import logger.Log;
import terminal.communication.StackStreamMessage;
import terminal.communication.StackStreamMessageParser;
import document.BTDesigner;
import document.Document;

public class StackStreamProcessor implements LineProcessor {

	private BTDesigner _designer;
	private StringBuilder _buffer = new StringBuilder();
	private StackStreamMessageParser _messageParser = new StackStreamMessageParser();

	public StackStreamProcessor(BTDesigner designer) {
		this._designer = designer;
	}
	
	@Override
	public void onEnd() {
		
	}

	@Override
	public void onNewLine(String line) {

		
		// end of message found: process the message data
		if (line.contains("---")) {
			StackStreamMessage message = new StackStreamMessage();
			
			String planID = this._buffer.substring(
					"data: ExeStack: changed : ".length(), this._buffer.indexOf(" code="));

			Document doc = _designer.getDocumentOfRunningPlan(planID);

			if (doc == null) {
				this._buffer = new StringBuilder();
				return;
			}
			
			// doc.cleanRunning();
			// doc.setRunning(Utils.getMatchedInstances(this.buffer, Utils.componentIdRegex));

			if (_messageParser.tryParse(_buffer.toString(), message)) {
				doc.onMessageReceive(message);
				Log.d("Message received: ");
				Log.d(message.toString());
			}
			else {
				Log.e("Failed to parse message from ros stack-stream\nBuffer: \n" + _buffer + "\n\n===============================\n");
			}
			
			this._buffer = new StringBuilder();
			return;
		}

		this._buffer.append(line);
		this._buffer.append('\n');
	}

	@Override
	public void onStart() {
	}

}