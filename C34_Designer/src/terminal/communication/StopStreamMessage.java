package terminal.communication;

import java.util.ArrayList;

public class StopStreamMessage implements IMessage<StopStreamMessage> {
	
	public enum PlanFinishReason {
		Success("OK"),
		Failure("FAILURE");
		
		private String _name;
		private PlanFinishReason(String name) { 
			_name = name; 
		}
		
		public static PlanFinishReason typeOf(String reason) {
			if (reason != null && reason.equalsIgnoreCase("ok"))
				return PlanFinishReason.Success;
			return PlanFinishReason.Failure;
		}
		
		@Override
		public String toString() {
			
			if (this == null) 
				return "";
			
			return _name;
		}
	}
	
	private PlanFinishReason _finishReason = PlanFinishReason.Success;
	private ArrayList<String> _tasksTree = new ArrayList<String>();
	private int _finishCode;
	private String _targetTaskId = "";
	private String _finishReasonDescription = "";

	public PlanFinishReason getFinishReason() {
		return _finishReason;
	}

	public void setFinishReason(PlanFinishReason finishReason) {
		this._finishReason = finishReason;
	}

	public void setFinishReason(String finishReason) {
		this._finishReason = PlanFinishReason.typeOf(finishReason);
	}
	
	public void setFinishReasonDescription(String finishReason) {
		if (finishReason != null)
			this._finishReasonDescription = finishReason;
	}
	
	public String getFinishReasonDescription() {
		return this._finishReasonDescription;
	}
	
	public int getFinishCode() {
		return this._finishCode;
	}
	
	public void setFinishCode(int finishCode) {
		this._finishCode = finishCode;
	}
	
	public String getTargetTaskId() {
		return _targetTaskId;
	}

	public void setTargetTaskId(String targetTaskId) {
		if (targetTaskId != null)
			this._targetTaskId = targetTaskId;
	}

	public ArrayList<String> getTasksTree() {
		return _tasksTree;
	}

	public void setFailedTasks(ArrayList<String> failedTasks) {
		this._tasksTree = failedTasks;
	}
	
	@Override
	public String toString() {
		return String.format(
				"Finish reason: %s\nLast task: %s",
				getFinishReason().toString(),
				getTargetTaskId()
				);
	}

	@Override
	public void clone(StopStreamMessage source) {
		// TODO Clone
		
	}
}
