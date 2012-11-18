package document;

import javax.swing.*;
import javax.swing.table.AbstractTableModel;
import javax.swing.event.ListSelectionEvent;
import javax.swing.event.ListSelectionListener;

import org.xml.sax.SAXException;

import document.listeners.PropertiesEditorAction;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.BorderLayout;
import java.awt.GridLayout;
import java.awt.Dimension;
import java.io.FileInputStream;
import java.util.Map;
import java.util.Properties;

public class PropertiesEditor extends JPanel implements ActionListener {
	private JTable table;
	private JCheckBox rowCheck;
	private JCheckBox columnCheck;
	private JCheckBox cellCheck;
	private ButtonGroup buttonGroup;
	private JTextArea output;
	private JPanel pnlButtons;
	private JButton btnSave;
	private JButton btnLoad;
	private JButton btnCancel;
	
	private final int KEY_INDEX = 0;
	private final int VALUE_INDEX = 1;

	public static String lastPath = "BTDesigner.xml";
	
	public PropertiesEditor(String path, JFrame frame) {
		super();
		setLayout(new BoxLayout(this, BoxLayout.Y_AXIS));

		table = new JTable(new PropertiesTableModel(path));

		table.setPreferredScrollableViewportSize(new Dimension(500, 270));
		table.setFillsViewportHeight(true);
		table.getSelectionModel().addListSelectionListener(new RowListener());
		table.getColumnModel().getSelectionModel()
				.addListSelectionListener(new ColumnListener());
		table.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);
		add(new JScrollPane(table));
		// add(new JLabel("Selection Mode"));
		// buttonGroup = new ButtonGroup();
		// addRadio("Multiple Interval Selection").setSelected(true);
		// addRadio("Single Selection");
		// addRadio("Single Interval Selection");
		//
		// add(new JLabel("Selection Options"));
		// rowCheck = addCheckBox("Row Selection");
		// rowCheck.setSelected(true);
		// columnCheck = addCheckBox("Column Selection");
		// cellCheck = addCheckBox("Cell Selection");
		// cellCheck.setEnabled(false);

		btnCancel = new JButton("Cancel");
		btnCancel.setActionCommand("cancel_properties");
		btnCancel.addActionListener(new PropertiesEditorAction(table, frame));
		btnLoad = new JButton("Load");
		btnLoad.setActionCommand("load_properties");
		btnLoad.addActionListener(new PropertiesEditorAction(table, frame));
		btnSave = new JButton("Save");
		btnSave.setActionCommand("save_properties");
		btnSave.addActionListener(new PropertiesEditorAction(table, frame));
		
		pnlButtons = new JPanel();
		pnlButtons.add(btnCancel, BorderLayout.WEST);
		pnlButtons.add(btnLoad, BorderLayout.CENTER);
		pnlButtons.add(btnSave, BorderLayout.EAST);
//		output = new JTextArea(5, 40);
//		output.setEditable(false);
//		add(new JScrollPane(output));
		add(pnlButtons);
	}

	private JCheckBox addCheckBox(String text) {
		JCheckBox checkBox = new JCheckBox(text);
		checkBox.addActionListener(this);
		add(checkBox);
		return checkBox;
	}

	private JRadioButton addRadio(String text) {
		JRadioButton b = new JRadioButton(text);
		b.addActionListener(this);
		buttonGroup.add(b);
		add(b);
		return b;
	}

	public void actionPerformed(ActionEvent event) {
		String command = event.getActionCommand();
		// Cell selection is disabled in Multiple Interval Selection
		// mode. The enabled state of cellCheck is a convenient flag
		// for this status.
		if ("Row Selection" == command) {
			table.setRowSelectionAllowed(rowCheck.isSelected());
			// In MIS mode, column selection allowed must be the
			// opposite of row selection allowed.
			if (!cellCheck.isEnabled()) {
				table.setColumnSelectionAllowed(!rowCheck.isSelected());
			}
		} else if ("Column Selection" == command) {
			table.setColumnSelectionAllowed(columnCheck.isSelected());
			// In MIS mode, row selection allowed must be the
			// opposite of column selection allowed.
			if (!cellCheck.isEnabled()) {
				table.setRowSelectionAllowed(!columnCheck.isSelected());
			}
		} else if ("Cell Selection" == command) {
			table.setCellSelectionEnabled(cellCheck.isSelected());
		} else if ("Multiple Interval Selection" == command) {
			table.setSelectionMode(ListSelectionModel.MULTIPLE_INTERVAL_SELECTION);
			// If cell selection is on, turn it off.
			if (cellCheck.isSelected()) {
				cellCheck.setSelected(false);
				table.setCellSelectionEnabled(false);
			}
			// And don't let it be turned back on.
			cellCheck.setEnabled(false);
		} else if ("Single Interval Selection" == command) {
			table.setSelectionMode(ListSelectionModel.SINGLE_INTERVAL_SELECTION);
			// Cell selection is ok in this mode.
			cellCheck.setEnabled(true);
		} else if ("Single Selection" == command) {
			table.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);
			// Cell selection is ok in this mode.
			cellCheck.setEnabled(true);
		}

		// Update checkboxes to reflect selection mode side effects.
		rowCheck.setSelected(table.getRowSelectionAllowed());
		columnCheck.setSelected(table.getColumnSelectionAllowed());
		if (cellCheck.isEnabled()) {
			cellCheck.setSelected(table.getCellSelectionEnabled());
		}
	}

//	private void outputSelection() {
//		output.append(String.format("Lead: %d, %d. ", table.getSelectionModel()
//				.getLeadSelectionIndex(), table.getColumnModel()
//				.getSelectionModel().getLeadSelectionIndex()));
//		output.append("Rows:");
//		for (int c : table.getSelectedRows()) {
//			output.append(String.format(" %d", c));
//		}
//		output.append(". Columns:");
//		for (int c : table.getSelectedColumns()) {
//			output.append(String.format(" %d", c));
//		}
//		output.append(".\n");
//	}

	private class RowListener implements ListSelectionListener {
		public void valueChanged(ListSelectionEvent event) {
			if (event.getValueIsAdjusting()) {
				return;
			}
//			output.append("ROW SELECTION EVENT. ");
//			outputSelection();
		}
	}

	private class ColumnListener implements ListSelectionListener {
		public void valueChanged(ListSelectionEvent event) {
			if (event.getValueIsAdjusting()) {
				return;
			}
//			output.append("COLUMN SELECTION EVENT. ");
//			outputSelection();
		}
	}

	class PropertiesTableModel extends AbstractTableModel {
		@Override
		public void fireTableDataChanged() {
			// TODO Auto-generated method stub
			buildTableFromProperties();
			super.fireTableDataChanged();
		}

		private String[] columnNames = { "Property", "Value" };

		// private Properties properties;
		
		private void buildTableFromProperties() {
			Map<String, String> map = PropertiesXmlHandler.getParameters();

			// build properties data matrix
			data = new Object[map.keySet().size()][2];
			int index = 0;
			for (Object key : map.keySet()) {
				data[index][KEY_INDEX] = new String(key.toString());
				data[index++][VALUE_INDEX] = new String(map.get(key
						.toString()));
			}
			
		}
		
		public PropertiesTableModel(String path) {

			// load
			try {
				PropertiesXmlHandler.loadAndSetProperties(path);
				buildTableFromProperties();
			} catch (Exception ex) {
				System.err.println("Error: " + ex.getMessage());
			}
		}

		private Object[][] data;

		public int getColumnCount() {
			return columnNames.length;
			// return data.length;
		}

		public int getRowCount() {
			return data.length;
			// return properties.keySet().size();
		}

		public String getColumnName(int col) {
			return columnNames[col];
		}

		public Object getValueAt(int row, int col) {
			System.out.println(String.format("<%d,%d>=%s", row, col, data[row][col]));
			return data[row][col];
		}

		/*
		 * JTable uses this method to determine the default renderer/ editor for
		 * each cell. If we didn't implement this method, then the last column
		 * would contain text ("true"/"false"), rather than a check box.
		 */
		// public Class getColumnClass(int c) {
		// return getValueAt(0, c).getClass();
		// }

		/*
		 * Don't need to implement this method unless your table's editable.
		 */
		public boolean isCellEditable(int row, int col) {
			// Note that the data/cell address is constant,
			// no matter where the cell appears onscreen.
			if (col == 1) {
				return true;
			} else {
				return false;
			}
		}

		/*
		 * Don't need to implement this method unless your table's data can
		 * change.
		 */
		public void setValueAt(Object value, int row, int col) {
			data[row][col] = value;
			fireTableCellUpdated(row, col);
		}

	}

	/**
	 * Create the GUI and show it. For thread safety, this method should be
	 * invoked from the event-dispatching thread.
	 */
	private static void createAndShowGUI() {
		// Disable boldface controls.
		UIManager.put("swing.boldMetal", Boolean.FALSE);

		// Create and set up the window.
		JFrame frame = new JFrame("Properties Editor");
		frame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);

		// Create and set up the content pane.
		PropertiesEditor newContentPane = new PropertiesEditor("BTDesigner.xml", frame);
		newContentPane.setOpaque(true); // content panes must be opaque
		frame.setContentPane(newContentPane);

		// Display the window.
		frame.pack();
		frame.setVisible(true);
	}
	
	public static void show(String path) {
		JFrame frame = new JFrame("Properties Editor");
		frame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);

		// Create and set up the content pane.
		PropertiesEditor newContentPane = new PropertiesEditor(path, frame);
		newContentPane.setOpaque(true); // content panes must be opaque
		frame.setContentPane(newContentPane);
		
		// Display the window.
		frame.pack();
		frame.setVisible(true);
	}
	
	public static void main(String[] args) throws Exception {
		// Schedule a job for the event-dispatching thread:
		// creating and showing this application's GUI.
		javax.swing.SwingUtilities.invokeLater(new Runnable() {
			public void run() {
				createAndShowGUI();
			}
		});
	}
}