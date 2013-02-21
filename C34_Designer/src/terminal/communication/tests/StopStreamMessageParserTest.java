package terminal.communication.tests;

import static org.junit.Assert.*;

import org.junit.Test;

import terminal.communication.StopStreamMessage.PlanFinishReason;
import terminal.communication.StopStreamMessage;
import terminal.communication.StopStreamMessageParser;

public class StopStreamMessageParserTest {

	StopStreamMessageParser _parser = new StopStreamMessageParser(); 

	@Test
	public void testParseSucceded() {
		String testInputString = 
				"data: ExeFinished: notification from GjxqfYOTVy\n" +
				"Unknown():OK;\n" + 
				" Sequence(Noname) [id=83176fee-cb02-4a0f-8434-b84de4d5e79d]:OK;";
		
		StopStreamMessage message = _parser.parse(testInputString);
		
		assertNotNull("Message is null", message);
		assertEquals(PlanFinishReason.Success, message.getFinishReason());
	}
	
	@Test
	public void testParseFailure() {
		String testInputString = 
				"data: ExeFinished: notification from EUrbCpdWWy\n" +
				"Unknown():FAILURE(1);\n" + 
				" Parallel(event1) [id=b056135a-e4e8-4ca6-8f02-86c0f16f03f6]:FAILURE(1);\n" + 
				"  Task(Monitor(target=LiveLock)) [id=00fa77be-648c-4c24-a20f-3c53a371179e]:FAILURE(1):Task server is not connected.;";
		
		StopStreamMessage message = _parser.parse(testInputString);
		
		assertNotNull("Message is null", message);
		assertEquals(PlanFinishReason.Failure, message.getFinishReason());
		assertEquals("00fa77be-648c-4c24-a20f-3c53a371179e", message.getTargetTaskId());
	}
}
