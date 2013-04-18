package terminal.lineprocessors;

import logger.Log;
import terminal.communication.CompactStackStreamMessageParser;
import terminal.communication.MessageParser;
import terminal.communication.StackStreamMessage;
import terminal.communication.StackStreamMessageParser;
import terminal.communication.UniversalStackStreamMessageParser;
import windows.designer.BTDesigner;
import document.Document;
import document.Parameters;

public class StackStreamProcessor implements LineProcessor {

	private BTDesigner _designer;
	private StringBuilder _buffer = new StringBuilder();
	private MessageParser<StackStreamMessage> _messageParser;

	public StackStreamProcessor(BTDesigner designer) {
		this._designer = designer;
		
//		if (Parameters.compact_stack_message) {
//			_messageParser = new CompactStackStreamMessageParser();
//			Log.i("STACKSTREAM", "Using compact stack stream message parser");
//		}
//		else {
//			_messageParser = new StackStreamMessageParser();
//			Log.i("STACKSTREAM", "Using regular stack stream message parser");
//		}
		_messageParser = new UniversalStackStreamMessageParser();
		Log.i("STACKSTREAM", "Using universal stack stream message parser");
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