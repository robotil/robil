package windows.lookuptable;

import javax.swing.table.AbstractTableModel;

import document.LookupTable;

public class LookupTableModel extends AbstractTableModel {

	private static final long serialVersionUID = -8635892421752741434L;

	private LookupTable _lookupTable;
	
	public LookupTableModel(LookupTable lookupTable) {
		this._lookupTable = lookupTable;
	}
	
	@Override
	public int getColumnCount() {
		return this._lookupTable.getColumnCount();
	}

	@Override
	public int getRowCount() {
		return this._lookupTable.getRowCount();
	}

	@Override
	public Object getValueAt(int rowIndex, int columnIndex) {
		return _lookupTable.getRow(rowIndex).get(columnIndex);
	}
	
	@Override
	public void setValueAt(Object aValue, int rowIndex, int columnIndex) {
		this._lookupTable.getRow(rowIndex).set(columnIndex, (String)aValue);
	}

	@Override
	public boolean isCellEditable(int rowIndex, int columnIndex) {
		return true;
	}
	
	@Override
	public String getColumnName(int column) {
		return this._lookupTable.getColumnNames()[column];
	}
	
}
