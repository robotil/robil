package terminal.communication.tests;

import static org.junit.Assert.*;
import junit.framework.Assert;

import org.junit.Test;

import terminal.communication.StackStreamMessage;
import terminal.communication.StackStreamMessage.ChangeType;
import terminal.communication.UniversalStackStreamMessageParser;

public class UniversalStackStreamMessageParserTest {

	UniversalStackStreamMessageParser _parser = new UniversalStackStreamMessageParser(); 

	@Test
	public void full_testParseTaskAllParametersFailure() {
		String testInputString = 
				"data: ExeStack: changed : P1 code=1, node=Task(PathPlanningFocus(x=1, y=2)) " +
				"[id=1d3b4a-345fcd3]($Task(PathPlanningFocus(x=1, y=2)) [id=1d3b4a-345fcd3]:FAILURE(1):Map is not ready$)";
		
		StackStreamMessage message = _parser.parse(testInputString);
		
		assertNotNull("Message is null", message);
		assertEquals("P1", message.getPlanLabel());
		assertEquals("Task", message.getTaskName());
		assertEquals("PathPlanningFocus(x=1, y=2)", message.getTaskParameters());
		assertEquals("1d3b4a-345fcd3", message.getTaskId());
		assertEquals(StackStreamMessage.TaskFinishReason.Failure, message.getTaskFinishReason());
		assertEquals(1, message.getTaskResultCode());
		assertEquals("Map is not ready", message.getTaskResultDescription());
		assertEquals(ChangeType.TaskFinished, message.getChangeType());
	}
	
	@Test
	public void full_testParseTaskFailure() {
		String testInputString = "data: ExeStack: changed : P1 code=1, " +
				"node=Sequence(S)($Sequence(S):FAILURE(1)$)(";
		
		StackStreamMessage message = _parser.parse(testInputString);
		
		assertNotNull("Message is null", message);
		assertEquals("P1", message.getPlanLabel());
		assertEquals("Sequence", message.getTaskName());
		assertEquals("S", message.getTaskParameters());
		assertEquals(ChangeType.TaskFinished, message.getChangeType());
	}
	
	@Test
	public void full_testParseTaskEmptyParameters() {
		String testInputString = "data: ExeStack: changed : P2 code=1, node=Unknown()($$)";
		
		StackStreamMessage message = _parser.parse(testInputString);
		
		assertNotNull("Message is null", message);
		assertEquals("P2", message.getPlanLabel());
		assertEquals("Unknown", message.getTaskName());
		assertEquals("", message.getTaskParameters());
		assertEquals(ChangeType.TaskFinished, message.getChangeType());
	}
	
	@Test
	public void full_testParseTaskSimple() {
		String testInputString = "data: ExeStack: changed : P2 code=0, node=Sequence(S)";
		
		StackStreamMessage message = _parser.parse(testInputString);
		
		assertNotNull("Message is null", message);
		assertEquals("P2", message.getPlanLabel());
		assertEquals("Sequence", message.getTaskName());
		assertEquals("S", message.getTaskParameters());
		assertEquals(ChangeType.TaskStarted, message.getChangeType());
	}
	
	@Test
	public void full_testParseTaskAdvanced() {
		String testInputString = "data: ExeStack: changed : P2 code=0, node=Task(PathPlanningFocus(x=1, y=2)) [id=PPF_ID]";

		StackStreamMessage message = _parser.parse(testInputString);
		
		assertNotNull("Message is null", message);
		assertEquals("P2", message.getPlanLabel());
		assertEquals("Task", message.getTaskName());
		assertEquals("PathPlanningFocus(x=1, y=2)", message.getTaskParameters());
		assertEquals("PPF_ID", message.getTaskId());
		assertEquals(ChangeType.TaskStarted, message.getChangeType());
	}

	@Test
	public void full_testParseTaskFailureCode() {
		String testInputString = "data: ExeStack: changed : P1 code=1, node=Sequence(S)($Sequence(S):FAILURE(1000)$)";
		
		StackStreamMessage message = _parser.parse(testInputString);
		
		assertNotNull("Message is null", message);
		assertEquals("P1", message.getPlanLabel());
		assertEquals("Sequence", message.getTaskName());
		assertEquals("S", message.getTaskParameters());
		assertEquals(1000, message.getTaskResultCode());
		assertEquals(ChangeType.TaskFinished, message.getChangeType());
	}
	
	@Test
	public void full_testParseTaskFailureNegativeCode() {
		String testInputString = "data: ExeStack: changed : P1 code=1, node=Sequence(S)($Sequence(S):FAILURE(-789)$)";
		
		StackStreamMessage message = _parser.parse(testInputString);
		
		assertNotNull("Message is null", message);
		assertEquals("P1", message.getPlanLabel());
		assertEquals("Sequence", message.getTaskName());
		assertEquals("S", message.getTaskParameters());
		assertEquals(-789, message.getTaskResultCode());
		assertEquals(ChangeType.TaskFinished, message.getChangeType());
	}
	
	@Test
	public void full_testParseTaskMultiLine() {
		String testInputString = 
				"data: ExeStack: changed : P1 code=1, node=Task(PathPlanning) [id=PP_ID]($Task(PathPlanning) [id=PP_ID]:FAILURE(1000)$)\n" +
				"plan{\n" +
				"  Unknown(){\n" +
				"    Sequence(S){\n" +
				"      Task(PathPlanning) [id=PP_ID]\n" +
				"    }\n" +
				"  }\n" +
				"}\n\n" +
				"---";
		
		StackStreamMessage message = _parser.parse(testInputString);
		
		assertNotNull("Message is null", message);
		assertEquals("P1", message.getPlanLabel());
		assertEquals("Task", message.getTaskName());
		assertEquals("PathPlanning", message.getTaskParameters());
		assertEquals(1000, message.getTaskResultCode());
		assertEquals("PP_ID", message.getTaskId());
		assertEquals(ChangeType.TaskFinished, message.getChangeType());
	}
	
	@Test
	public void compact_fullMessageParseTest() {
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
	public void compact_withoutDescriptionMessageParseTest() {
		StackStreamMessage message = new StackStreamMessage();
		String inputString = "0:1d3b4a-345fcd3:3F2504E0-4F89-11D3-9A0C-0305E82C3301:1000";
		
		Assert.assertTrue(_parser.tryParse(inputString, message));
		Assert.assertEquals(0, message.getChangeType().getCode());
		Assert.assertEquals("1d3b4a-345fcd3", message.getPlanLabel());
		Assert.assertEquals("3F2504E0-4F89-11D3-9A0C-0305E82C3301", message.getTaskId());
		Assert.assertEquals(1000, message.getTaskResultCode());
	}
	
	@Test
	public void compact_withoutReturnValueMessageParseTest() {
		StackStreamMessage message = new StackStreamMessage();
		String inputString = "0:1d3b4a-345fcd3:3F2504E0-4F89-11D3-9A0C-0305E82C3301";
		
		Assert.assertTrue(_parser.tryParse(inputString, message));
		Assert.assertEquals(0, message.getChangeType().getCode());
		Assert.assertEquals("1d3b4a-345fcd3", message.getPlanLabel());
		Assert.assertEquals("3F2504E0-4F89-11D3-9A0C-0305E82C3301", message.getTaskId());
	}
	
}
