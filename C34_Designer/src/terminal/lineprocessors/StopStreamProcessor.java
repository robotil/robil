package terminal.lineprocessors;

import java.util.regex.Matcher;
import java.util.regex.Pattern;

import terminal.communication.StopStreamMessage;
import terminal.communication.StopStreamMessageParser;

import logger.Log;
import document.BTDesigner;
import document.Document;

public class StopStreamProcessor implements LineProcessor {
	
	private BTDesigner _designer;
	private StringBuilder _buffer = new StringBuilder();
	private Pattern _planIdPattern = Pattern.compile("data: ExeFinished: notification from (\\w+)");
	private StopStreamMessageParser _messageParser = new StopStreamMessageParser();
	
	public StopStreamProcessor(BTDesigner designer) {
		this._designer = designer;
	}
	
	private Document extractDocument(String message) {
		Matcher planIdMatcher = _planIdPattern.matcher(_buffer.toString());
		
		if (planIdMatcher.find()) {
			String planId = planIdMatcher.group(1);
			Document doc = _designer.getDocumentOfRunningPlan(planId);
			return doc;
		}
		
		return null;
	}
	
	@Override
	public void onEnd() { }

	@Override
	public void onNewLine(String line) {
		
		if (line.contains("---")) {
			Log.i("STOPSTREAM", "Message received");
			Log.i("STOPSTREAM", _buffer.toString());
			
			Document document = extractDocument(_buffer.toString());
			
			if (document == null) {
				Log.i("STOPSTREAM", "Document not found");
				_buffer = new StringBuilder();
				return ;
			}
			
			StopStreamMessage message = _messageParser.parse(_buffer.toString());
			Log.i("STOPSTREAM","Message parsed");
			document.onStop(message);
			
			// Clean buffer
			_buffer = new StringBuilder();
			return;
		}
		
		_buffer.append(line);
		_buffer.append("\n");
	}

	@Override
	public void onStart() { }
	
}