package windows.filemanager.filelistproviders;

import java.util.Date;

public class FileDescription {
	
	private static String[] _fieldNames = { "Filename", "Modify date" };
	
	private String _filename;
	private Date _modifyDate;
	private Date _createDate;
	private boolean _directory;
	
	public static String[] getFieldNames() {
		return _fieldNames;
	}
	
	public static int getFieldCount() {
		return getFieldNames().length;
	}
	
	public String getField(int index) {
		switch (index) {
		case 0:
			return getFilename();
		case 1:
			return getModifyDateString();
		case 2:
			return getCreateDateString();
		}
		
		return "";
	}
	
	public void setDirectory(boolean isDirectory) {
		this._directory = isDirectory;
	}
	
	public boolean isDirectory() {
		return this._directory;
	}
	
	public String getFilename() {
		return _filename;
	}
	
	public void setFilename(String filename) {
		this._filename = filename;
	}

	public Date getModifyDate() {
		return _modifyDate;
	}
	
	public String getModifyDateString() {
		if (getModifyDate() == null)
			return "";
		return getModifyDate().toString();
	}

	public void setModifyDate(Date modifyDate) {
		this._modifyDate = modifyDate;
	}

	public Date getCreateDate() {
		return _createDate;
	}
	
	public String getCreateDateString() {
		if (getCreateDate() == null)
			return "";
		return getCreateDate().toString();
	}

	public void setCreateDate(Date createDate) {
		this._createDate = createDate;
	}
	
}
