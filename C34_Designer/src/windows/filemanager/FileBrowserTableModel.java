package windows.filemanager;

import javax.swing.ImageIcon;
import javax.swing.table.AbstractTableModel;

import document.ui.Utilities;

import windows.filemanager.filelistproviders.FileDescription;

public class FileBrowserTableModel extends AbstractTableModel {
	private static final long serialVersionUID = 1L;

	private FileDescription[] _files;
	
	public FileBrowserTableModel() {
		this._files = new FileDescription[0];
	}
	
	public FileBrowserTableModel(FileDescription[] files) {
		this._files = files;
	}
	
	@Override
	public String getColumnName(int column) {
		return super.getColumnName(column);
	}
	
	@Override
	public int getColumnCount() {
		return FileDescription.getFieldCount() + 1; // +1 for icon
	}

	@Override
	public int getRowCount() {
		return _files.length;
	}

	@Override
	public Object getValueAt(int rowIndex, int columnIndex) {
		
		if (columnIndex == 0) {
			// Icon
			return Utilities.loadIcon("source.png");
		}
		
		return this._files[rowIndex].getField(columnIndex - 1); // -1 because of icon at 0
	}
	
	@Override
	public Class<?> getColumnClass(int columnIndex) {
		if (columnIndex == 0)
			return ImageIcon.class;
		
		return super.getColumnClass(columnIndex);
	}
}
