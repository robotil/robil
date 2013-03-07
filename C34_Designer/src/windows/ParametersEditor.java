package windows;

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;

import javax.swing.BoxLayout;
import javax.swing.ButtonGroup;
import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.JRadioButton;
import javax.swing.JScrollPane;
import javax.swing.JTable;
import javax.swing.JTextArea;
import javax.swing.ListSelectionModel;
import javax.swing.UIManager;
import javax.swing.event.ListSelectionEvent;
import javax.swing.event.ListSelectionListener;
import javax.swing.table.AbstractTableModel;

import logger.Log;

import document.ParametersXmlHandler;
import document.actions.PropertiesEditorAction;

public class ParametersEditor extends JPanel implements ActionListener {
	private class ColumnListener implements ListSelectionListener {
		@Override
		public void valueChanged(ListSelectionEvent event) {
			if (event.getValueIsAdjusting()) {
				return;
			}
			// output.append("COLUMN SELECTION EVENT. ");
			// outputSelection();
		}
	}

	class PropertiesTableModel extends AbstractTableModel {
		private static final long serialVersionUID = 4416884143948906001L;

		private String[] columnNames = { "Property", "Value" };

		private Object[][] data;

		// private Properties properties;

		public PropertiesTableModel(String path) {

			// load
			try {
				ParametersXmlHandler.loadAndSetProperties(path);
				buildTableFromProperties();
			} catch (Exception ex) {
				Log.e("Error: " + ex.getMessage());
			}
		}

		private void buildTableFromProperties() {
			Map<String, String> map = ParametersXmlHandler.getParameters();

			// build properties data matrix
			this.data = new Object[map.keySet().size()][2];
			int index = 0;
			List<String> keyset = new ArrayList<String>();
			keyset.addAll(map.keySet());
			Collections.sort(keyset);
			for (Object key : keyset) {
				this.data[index][ParametersEditor.this.KEY_INDEX] = new String(
						key.toString());
				this.data[index++][ParametersEditor.this.VALUE_INDEX] = new String(
						map.get(key.toString()));
			}

		}

		@Override
		public void fireTableDataChanged() {
			// TODO Auto-generated method stub
			buildTableFromProperties();
			super.fireTableDataChanged();
		}

		@Override
		public int getColumnCount() {
			return this.columnNames.length;
			// return data.length;
		}

		@Override
		public String getColumnName(int col) {
			return this.columnNames[col];
		}

		@Override
		public int getRowCount() {
			return this.data.length;
			// return properties.keySet().size();
		}

		@Override
		public Object getValueAt(int row, int col) {
			//Log.d(String.format("<%d,%d>=%s", row, col,this.data[row][col]));
			return this.data[row][col];
		}

		/*
		 * Don't need to implement this method unless your table's editable.
		 */
		@Override
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
		@Override
		public void setValueAt(Object value, int row, int col) {
			this.data[row][col] = value;
			fireTableCellUpdated(row, col);
		}

	}

	private class RowListener implements ListSelectionListener {
		@Override
		public void valueChanged(ListSelectionEvent event) {
			if (event.getValueIsAdjusting()) {
				return;
			}
			// output.append("ROW SELECTION EVENT. ");
			// outputSelection();
		}
	}

	/**
	 * 
	 */
	private static final long serialVersionUID = -6808008693892056957L;

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
		ParametersEditor newContentPane = new ParametersEditor(
				"BTDesigner.xml", frame);
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
			@Override
			public void run() {
				createAndShowGUI();
			}
		});
	}

	public static void show(String path) {
		JFrame frame = new JFrame("Properties Editor");
		frame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);

		// Create and set up the content pane.
		ParametersEditor newContentPane = new ParametersEditor(path, frame);
		newContentPane.setOpaque(true); // content panes must be opaque
		frame.setContentPane(newContentPane);

		// Display the window.
		frame.pack();
		frame.setVisible(true);
	}

	private JTable table;
	private JCheckBox rowCheck;
	private JCheckBox columnCheck;
	private JCheckBox cellCheck;

	private ButtonGroup buttonGroup;
	@SuppressWarnings("unused")
	private JTextArea output;

	private JPanel pnlButtons;

	private JButton btnSave;

	private JButton btnLoad;

	private JButton btnCancel;

	private final int KEY_INDEX = 0;

	private final int VALUE_INDEX = 1;

	public ParametersEditor(String path, JFrame frame) {
		super();
		setLayout(new BoxLayout(this, BoxLayout.Y_AXIS));

		this.table = new JTable(new PropertiesTableModel(path));

		this.table.setPreferredScrollableViewportSize(new Dimension(500, 200));
		this.table.setFillsViewportHeight(true);
		this.table.getSelectionModel().addListSelectionListener(
				new RowListener());
		this.table.getColumnModel().getSelectionModel()
				.addListSelectionListener(new ColumnListener());
		this.table.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);
		add(new JScrollPane(this.table));

		//BUTTONS
		this.btnCancel = new JButton("Cancel");
		this.btnCancel.setActionCommand("cancel_properties");
		this.btnCancel.addActionListener(new PropertiesEditorAction(this.table,
				frame));
		this.btnLoad = new JButton("Load");
		this.btnLoad.setActionCommand("load_properties");
		this.btnLoad.addActionListener(new PropertiesEditorAction(this.table,
				frame));
		this.btnSave = new JButton("Save");
		this.btnSave.setActionCommand("save_properties");
		this.btnSave.addActionListener(new PropertiesEditorAction(this.table,
				frame));

		this.pnlButtons = new JPanel();
		this.pnlButtons.add(this.btnCancel, BorderLayout.WEST);
		this.pnlButtons.add(this.btnLoad, BorderLayout.CENTER);
		this.pnlButtons.add(this.btnSave, BorderLayout.EAST);

		add(this.pnlButtons);

	}

	@Override
	public void actionPerformed(ActionEvent event) {
		String command = event.getActionCommand();
		// Cell selection is disabled in Multiple Interval Selection
		// mode. The enabled state of cellCheck is a convenient flag
		// for this status.
		if ("Row Selection" == command) {
			this.table.setRowSelectionAllowed(this.rowCheck.isSelected());
			// In MIS mode, column selection allowed must be the
			// opposite of row selection allowed.
			if (!this.cellCheck.isEnabled()) {
				this.table.setColumnSelectionAllowed(!this.rowCheck.isSelected());
			}
		} else if ("Column Selection" == command) {
			this.table.setColumnSelectionAllowed(this.columnCheck.isSelected());
			// In MIS mode, row selection allowed must be the
			// opposite of column selection allowed.
			if (!this.cellCheck.isEnabled()) {
				this.table.setRowSelectionAllowed(!this.columnCheck.isSelected());
			}
		} else if ("Cell Selection" == command) {
			this.table.setCellSelectionEnabled(this.cellCheck.isSelected());
		} else if ("Multiple Interval Selection" == command) {
			this.table.setSelectionMode(ListSelectionModel.MULTIPLE_INTERVAL_SELECTION);
			// If cell selection is on, turn it off.
			if (this.cellCheck.isSelected()) {
				this.cellCheck.setSelected(false);
				this.table.setCellSelectionEnabled(false);
			}
			// And don't let it be turned back on.
			this.cellCheck.setEnabled(false);
		} else if ("Single Interval Selection" == command) {
			this.table.setSelectionMode(ListSelectionModel.SINGLE_INTERVAL_SELECTION);
			// Cell selection is ok in this mode.
			this.cellCheck.setEnabled(true);
		} else if ("Single Selection" == command) {
			this.table.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);
			// Cell selection is ok in this mode.
			this.cellCheck.setEnabled(true);
		}

		// Update checkboxes to reflect selection mode side effects.
		this.rowCheck.setSelected(this.table.getRowSelectionAllowed());
		this.columnCheck.setSelected(this.table.getColumnSelectionAllowed());
		if (this.cellCheck.isEnabled()) {
			this.cellCheck.setSelected(this.table.getCellSelectionEnabled());
		}
	}

	@SuppressWarnings("unused")
	private JCheckBox addCheckBox(String text) {
		JCheckBox checkBox = new JCheckBox(text);
		checkBox.addActionListener(this);
		add(checkBox);
		return checkBox;
	}

	@SuppressWarnings("unused")
	private JRadioButton addRadio(String text) {
		JRadioButton b = new JRadioButton(text);
		b.addActionListener(this);
		this.buttonGroup.add(b);
		add(b);
		return b;
	}
}