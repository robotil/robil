package document;

import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;

import elements.tasks.Task;

public class PlanExecution {
	private Date _startTime;
	private Date _finishTime;
	private boolean _failed;
	private Task _lastTask;
	private ArrayList<Task> _tasksList;
	private boolean _aborted;

	public PlanExecution() {
		start();
	}
	
	public PlanExecution(Task lastTask, ArrayList<Task> tasksList, boolean failed, Date startTime, boolean aborted) {
		start(startTime);
		stop(lastTask, tasksList, failed, aborted);
	}
	
	public void start() {
		this._startTime = new Date();
	}
	
	public void start(Date startTime) {
		this._startTime = startTime;
	}
	
	public void stop(Task lastTask, ArrayList<Task> tasksList, boolean failed, boolean aborted) {
		this._lastTask = lastTask;
		this._failed = failed;
		this._finishTime = new Date();
		this._aborted = aborted;
		this._tasksList = tasksList;
	}
	
	public void setStartTime(Date time) {
		this._startTime = time;
	}
		
	public Date getStartTime() {
		return this._startTime;
	}
	
	public Date getFinishTime() {
		return this._finishTime;
	}

	public boolean isFailure() {
		return _failed;
	}
	
	public Task getLastTask() {
		return this._lastTask;
	}
	
	public boolean getAborted() {
		return _aborted;
	}
	
	@Override
	public String toString() {
		SimpleDateFormat dateFormat = new SimpleDateFormat("HH:mm:ss");
		return String.format(
				"[%s-%s] %s", 
				dateFormat.format(getStartTime()),
				dateFormat.format(getFinishTime()),
				isFailure() ? "FAILURE" : "SUCCESS");
	}
	
	public String toStringDetailed() {
		StringBuilder result = new StringBuilder();
		
		int tabs = 1;
		
		if (_tasksList != null)
			for (Task task : _tasksList) {
				for (int i = 0; i < tabs; i++) 
					result.append("-");
				
				result.append(task.getText());
				result.append("\n");
				tabs++;
			}
			
		return toString() + "\n" + result;
	}
}
