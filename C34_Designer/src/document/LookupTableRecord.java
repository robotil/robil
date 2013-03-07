package document;

import org.w3c.dom.Element;

public class LookupTableRecord {
	private String _taskName;
	private String _type;
	private String _planner;
	private String _fileName;
	
	public LookupTableRecord() {
		this("", "", "", "");
	}
	
	public LookupTableRecord(String taskName, String type, String planner, String fileName) {
		setTaskName(taskName);
		setType(type);
		setPlanner(planner);
		setFileName(fileName);
	}
	
	public static String[] getTypeValues() {
		return new String[] { "local" };
	}
	
	public static String[] getPlannerValues() {
		return new String[] { "file" };
	}
	
	public String getTaskName() {
		return _taskName;
	}
	public void setTaskName(String taskName) {
		if (taskName == null)
			return;
		
		this._taskName = taskName;
	}
	public String getType() {
		return _type;
	}
	public void setType(String type) {
		if (type == null)
			return;
		
		this._type = type;
	}
	public String getPlanner() {
		return _planner;
	}
	public void setPlanner(String planner) {
		if (planner == null)
			return;
		
		this._planner = planner;
	}
	public String getFileName() {
		return _fileName;
	}
	public void setFileName(String fileName) {
		if (fileName == null)
			return;
		
		this._fileName = fileName;
	}
	
	public void set(int index, String value) {
		switch (index) {
		case 0:
			setTaskName(value);
			break;
		case 1:
			setType(value);
			break;
		case 2:
			setPlanner(value);
			break;
		case 3:
			setFileName(value);
			break;
		}
	}
	
	public String get(int index) {
		switch (index) {
		case 0:
			return getTaskName();
		case 1:
			return getType();
		case 2:
			return getPlanner();
		case 3:
			return getFileName();
		default:
			return "BADINDEX";
		}
	}

	public Element getXmlElement(org.w3c.dom.Document document) {
		Element element = document.createElement("task");
		element.setAttribute("name", getTaskName());
		element.setAttribute("type", getType());
		element.setAttribute("planner", getPlanner());
		element.setAttribute("file-name", getFileName());
		return element;
	}
}