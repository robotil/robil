package terminal.communication.tests;

import org.junit.Assert;
import org.junit.Test;

import terminal.communication.CompactStackStreamMessageParser;
import terminal.communication.MessageParser;
import terminal.communication.StackStreamMessage;

public class CompactStackStreamMessageParserTest {
	MessageParser<StackStreamMessage> _parser = new CompactStackStreamMessageParser();
	
	@Test
	public void fullMessageParseTest() {
		StackStreamMessage message = new StackStreamMessage();
		String inputString = "0:1d3b4a-345fcd3:3F2504E0-4F89-11D3-9A0C-0305E82C3301:1000:Something bad happend";
		
		Assert.assertTrue(_parser.tryParse(inputString, message));
		Assert.assertEquals(0, message.getChangeType().getCode());
		Assert.assertEquals("1d3b4a-345fcd3", message.getPlanLabel());
		Assert.assertEquals("3F2504E0-4F89-11D3-9A0C-0305E82C3301", message.getTaskId());
		Assert.assertEquals(1000, message.getTaskResultCode());
		Assert.assertEquals("Something bad happend", message.getTaskResultDescription());
	}
	
	@Test
	public void withoutDescriptionMessageParseTest() {
		StackStreamMessage message = new StackStreamMessage();
		String inputString = "0:1d3b4a-345fcd3:3F2504E0-4F89-11D3-9A0C-0305E82C3301:1000";
		
		Assert.assertTrue(_parser.tryParse(inputString, message));
		Assert.assertEquals(0, message.getChangeType().getCode());
		Assert.assertEquals("1d3b4a-345fcd3", message.getPlanLabel());
		Assert.assertEquals("3F2504E0-4F89-11D3-9A0C-0305E82C3301", message.getTaskId());
		Assert.assertEquals(1000, message.getTaskResultCode());
	}
	
	@Test
	public void withoutReturnValueMessageParseTest() {
		StackStreamMessage message = new StackStreamMessage();
		String inputString = "0:1d3b4a-345fcd3:3F2504E0-4F89-11D3-9A0C-0305E82C3301";
		
		Assert.assertTrue(_parser.tryParse(inputString, message));
		Assert.assertEquals(0, message.getChangeType().getCode());
		Assert.assertEquals("1d3b4a-345fcd3", message.getPlanLabel());
		Assert.assertEquals("3F2504E0-4F89-11D3-9A0C-0305E82C3301", message.getTaskId());
	}
}
