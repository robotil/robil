package elements.tasks;

import java.text.SimpleDateFormat;
import java.util.Date;

/**
 * Holds run result
 * @author blackpc
 */
public class TaskResult {
	private int _code = 0;
	private String _description = "";
	private Date _startTime;
	private Date _finishTime;
	private boolean _failed;
	
	public TaskResult() { this(0, ""); }
	
	public TaskResult(int code) { this(code, ""); }
	
	public TaskResult(int code, String description) {
		_code = code;
		_description = description;
		_finishTime = new Date();
		_failed = _code > 0;
	}
	
	public int getCode() {
		return _code;
	}
	
	public void setCode(int code) {
		this._code = code;
	}
	
	public String getDescription() {
		return _description;
	}
	
	public void setDescription(String description) {
		this._description = description;
	}
	
	public Date getFinishTime() {
		return this._finishTime;
	}

	public void setStartTime(Date time) {
		this._startTime = time;
	}
		
	public Date getStartTime() {
		return this._startTime;
	}

	public boolean isFailure() {
		return _failed;
	}
	
	@Override
	public String toString() {
		SimpleDateFormat dateFormat = new SimpleDateFormat("HH:mm:ss");
		return String.format(
				"[%s-%s] %s(%d) : %s", 
				dateFormat.format(getStartTime()),
				dateFormat.format(getFinishTime()),
				isFailure() ? "FAILURE" : "SUCCESS",
				getCode(), 
				getDescription());
	}
}