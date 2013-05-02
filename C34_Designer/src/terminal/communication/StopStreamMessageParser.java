package terminal.communication;

import java.util.ArrayList;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class StopStreamMessageParser {

	private final Pattern _planFinishReasonPattern = Pattern.compile("\\):(OK|FAILURE)\\((\\d+)\\)(:[^;]*)?");
	private final Pattern _planIdsPattern = Pattern.compile("\\[id\\=([\\w\\-]+)\\]");
	
	public StopStreamMessage parse(String inputString) {
		StopStreamMessage message = new StopStreamMessage();
		
		Matcher planFinishMatcher = _planFinishReasonPattern.matcher(inputString);
		
		if (planFinishMatcher.find()) {
			message.setFinishReason(planFinishMatcher.group(1));
			
			if (planFinishMatcher.group(2) != null)
				message.setFinishCode(Integer.parseInt(planFinishMatcher.group(2)));
			
			if (planFinishMatcher.group(3) != null)
				message.setFinishReasonDescription(planFinishMatcher.group(3).replace(":", ""));
		}
		
		
		
		Matcher planIdsMatcher = _planIdsPattern.matcher(inputString);
		ArrayList<String> planIds = new ArrayList<String>();
		while (planIdsMatcher.find()) 
			planIds.add(planIdsMatcher.group(1));
		message.setFailedTasks(planIds);
				
		if (planIds.size() > 0)
			message.setTargetTaskId(planIds.get(planIds.size() - 1));
		
		return message;
	}
}
