package windows.lookuptable;

import java.awt.Component;
import java.awt.Dimension;
import java.awt.EventQueue;
import java.awt.FileDialog;
import java.awt.Frame;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.Insets;
import java.awt.Point;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.BorderFactory;
import javax.swing.DefaultCellEditor;
import javax.swing.JButton;
import javax.swing.JComboBox;
import javax.swing.JComponent;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JPopupMenu;
import javax.swing.JScrollPane;
import javax.swing.JTable;
import javax.swing.JTextField;
import javax.swing.UIManager;
import javax.swing.border.TitledBorder;
import javax.swing.table.TableCellRenderer;

import document.LookupTable;
import document.LookupTableRow;

public class LookupTableEditor extends JFrame {

	private static final long serialVersionUID = 507538166623467081L;
	private LookupTable _lookupTable;
	private JTable _jTable;

	public LookupTableEditor(Frame parent, String fileName) {
		_lookupTable = new LookupTable(fileName);
		init(parent);
	}

	private void init(Frame parent) {
		setLocationRelativeTo(parent);
		setSize(new Dimension(800, 400));
		buildWindow();
		setVisible(true);
	}
	
	@SuppressWarnings({ "rawtypes", "unchecked" })
	private JPanel createNewRowPanel() {
		JPanel panel = new JPanel();
		panel.setLayout(new GridBagLayout());
		GridBagConstraints c = new GridBagConstraints();
		panel.setBorder(new TitledBorder("Add new record"));
		
		final JTextField taskName = new JTextField();
		final JComboBox type = new JComboBox(LookupTableRow.getTypeValues());
		final JComboBox planner = new JComboBox(LookupTableRow.getPlannerValues());
		final JTextField fileName = new JTextField();
		
		c = new GridBagConstraints();
		c.insets = new Insets(0, 10, 10, 10);
		c.fill = GridBagConstraints.HORIZONTAL;
		c.gridx = 0; c.gridy = 0;
		c.weightx = 0.15; c.weighty = 0.0;
		panel.add(new JLabel("Task name"), c);
		
		c = new GridBagConstraints();
		c.insets = new Insets(0, 10, 10, 10);
		c.fill = GridBagConstraints.HORIZONTAL;
		c.gridx = 1; c.gridy = 0;
		c.weightx = 0.15; c.weighty = 0.0;
		panel.add(new JLabel("Type"), c);
		
		c = new GridBagConstraints();
		c.insets = new Insets(0, 10, 10, 10);
		c.fill = GridBagConstraints.HORIZONTAL;
		c.gridx = 2; c.gridy = 0;
		c.weightx = 0.15; c.weighty = 0.0;
		panel.add(new JLabel("Planner"), c);
		
		c = new GridBagConstraints();
		c.insets = new Insets(0, 10, 10, 10);
		c.fill = GridBagConstraints.HORIZONTAL;
		c.gridx = 3; c.gridy = 0;
		c.weightx = 0.4; c.weighty = 0.0;
		panel.add(new JLabel("Filename"), c);
		
		c = new GridBagConstraints();
		c.insets = new Insets(0, 10, 10, 10);
		c.fill = GridBagConstraints.HORIZONTAL;
		c.gridx = 4; c.gridy = 0;
		c.weightx = 0.0; c.weighty = 0.0;
		panel.add(new JLabel(""), c);
		
		c = new GridBagConstraints();
		c.insets = new Insets(0, 10, 10, 10);
		c.fill = GridBagConstraints.HORIZONTAL;
		c.gridx = 5; c.gridy = 0;
		c.weightx = 0.15; c.weighty = 0.0;
		panel.add(new JLabel(""), c);
		
		
		
		c = new GridBagConstraints();
		c.insets = new Insets(0, 10, 10, 10);
		c.fill = GridBagConstraints.HORIZONTAL;
		c.gridx = 0; c.gridy = 1;
		c.weightx = 0.0; c.weighty = 0.0;
		panel.add(taskName, c);
		
		c = new GridBagConstraints();
		c.insets = new Insets(0, 10, 10, 10);
		c.fill = GridBagConstraints.HORIZONTAL;
		c.gridx = 1; c.gridy = 1;
		c.weightx = 0.0; c.weighty = 0.0;
		panel.add(type, c);
		
		c = new GridBagConstraints();
		c.insets = new Insets(0, 10, 10, 10);
		c.fill = GridBagConstraints.HORIZONTAL;
		c.gridx = 2; c.gridy = 1;
		c.weightx = 0.0; c.weighty = 0.0;
		panel.add(planner, c);
		
		c = new GridBagConstraints();
		c.insets = new Insets(0, 10, 10, 0);
		c.fill = GridBagConstraints.HORIZONTAL;
		c.gridx = 3; c.gridy = 1;
		c.weightx = 1.0; c.weighty = 0.0;
		panel.add(fileName, c);
		
		c = new GridBagConstraints();
		c.insets = new Insets(0, 0, 10, 10);
		c.fill = GridBagConstraints.HORIZONTAL;
		c.gridx = 4; c.gridy = 1;
		c.weightx = 0.0; c.weighty = 0.0;
		panel.add(new JButton("Browse...") {
			private static final long serialVersionUID = 1L;
			{
				addActionListener(new ActionListener() {
					@Override
					public void actionPerformed(ActionEvent arg) {
						FileDialog dialog = new FileDialog(LookupTableEditor.this, "Browse file", FileDialog.SAVE);
						dialog.setDirectory("~");
						dialog.setVisible(true);
						
						if (dialog.getFile() != null)
							fileName.setText(dialog.getDirectory() + dialog.getFile());
					}
				});
			}
		}, c);
		
		c = new GridBagConstraints();
		c.insets = new Insets(0, 10, 10, 10);
		c.fill = GridBagConstraints.HORIZONTAL;
		c.gridx = 5; c.gridy = 1;
		c.weightx = 0.0; c.weighty = 0.0;
		panel.add(new JButton("Add") {
			private static final long serialVersionUID = -464974929293997291L;
			{
				addActionListener(new ActionListener() {
					@Override
					public void actionPerformed(ActionEvent arg) {
						_lookupTable.add(taskName.getText(), type.getSelectedItem().toString(), planner.getSelectedItem().toString(), fileName.getText());
						
						EventQueue.invokeLater(new Runnable() {
							@Override
							public void run() {
								_jTable.updateUI();
								_jTable.repaint();
							}
						});
					}
				});
			}
		}, c);
		
		return panel;
	}
	
	@SuppressWarnings({ "unchecked", "rawtypes" })
	private void buildWindow() {
		UIManager.put("TextArea.margin", new Insets(10, 10, 10, 10));
		setLayout(new GridBagLayout());
		GridBagConstraints c = new GridBagConstraints();
		
		c.insets = new Insets(10, 10, 10, 10);
		c.fill = GridBagConstraints.HORIZONTAL;
		c.gridx = 0;
		c.gridy = 0;
		c.weightx = 1.0;
		c.weighty = 0.0;
		add(new JLabel("Filename: " + this._lookupTable.getFilename()), c);
		
		c = new GridBagConstraints();
		c.insets = new Insets(0, 10, 10, 10);
		c.fill = GridBagConstraints.BOTH;
		c.gridx = 0;
		c.gridy = 1;
		c.weightx = 1.0;
		c.weighty = 1.0;
		_jTable = new JTable(new LookupTableModel(_lookupTable)) {
			private static final long serialVersionUID = 1L;
			@Override
			public JPopupMenu getComponentPopupMenu() {
				Point mousePosition = getMousePosition();
				
				if (mousePosition != null && rowAtPoint(mousePosition) >= 0) 
					return new LookupTablePopupMenu(_lookupTable, rowAtPoint(mousePosition), this);
				else
					return null;
			}
			@Override
			public Component prepareRenderer(TableCellRenderer renderer,
					int row, int column) {
				Component comp = super.prepareRenderer(renderer, row, column);
				if (JComponent.class.isInstance(comp)){
		            ((JComponent)comp).setBorder(BorderFactory.createEmptyBorder(5, 5, 5, 5));
		        }
				return comp;
			}
		};
		_jTable.getColumnModel().getColumn(1).setCellEditor(new DefaultCellEditor(new JComboBox(LookupTableRow.getTypeValues())));
		_jTable.getColumnModel().getColumn(2).setCellEditor(new DefaultCellEditor(new JComboBox(LookupTableRow.getPlannerValues())));
		_jTable.setRowHeight(28);
		
		
		add(new JScrollPane(_jTable), c);
		
		c = new GridBagConstraints();
		c.insets = new Insets(0, 10, 10, 10);
		c.fill = GridBagConstraints.HORIZONTAL;
		c.gridx = 0;
		c.gridy = 2;
		c.weightx = 0.0;
		c.weighty = 0.0;	
		add(createNewRowPanel(), c);
		
		
		c = new GridBagConstraints();
		c.insets = new Insets(0, 10, 10, 10);
		c.fill = GridBagConstraints.NONE;
		c.anchor = GridBagConstraints.EAST;
		c.gridx = 0;
		c.gridy = 3;
		c.weightx = 0.0;
		c.weighty = 0.0;
		add(new JButton("Save") {
			private static final long serialVersionUID = 1L;
			{
				addActionListener(new ActionListener() {
					@Override
					public void actionPerformed(ActionEvent arg0) {
						_lookupTable.save();
						dispose();						
					}
				});
			}
		}, c);
		
		setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
	}
	
	
}
