package elements.tasks;

import java.awt.Dimension;
import java.awt.Insets;
import java.awt.KeyEventDispatcher;
import java.awt.KeyboardFocusManager;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.KeyAdapter;
import java.awt.event.KeyEvent;
import java.awt.event.WindowEvent;
import java.awt.event.WindowListener;
import java.util.Vector;

import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JComboBox;
import javax.swing.JDialog;
import javax.swing.JLabel;
import javax.swing.JOptionPane;
import javax.swing.JScrollPane;
import javax.swing.JTextArea;
import javax.swing.JTextField;
import javax.swing.ScrollPaneConstants;
import javax.swing.UIManager;

import logger.Log;

import document.description.TaskDescription;
import elements.JAutoSuggestComboBox;

@SuppressWarnings("rawtypes")
class ModifyDialog extends JDialog {
	private static final long serialVersionUID = 1739783395697186997L;
	private Task _task;
	
	JAutoSuggestComboBox txtName = null;
	JComboBox cType = null;

	JTextField txtDbgTime = null;
	JComboBox txtDbgResult = null;
	JCheckBox chkCollapse = null;
	JTextArea txtTaskDescAlgoritm;
	JScrollPane txtTaskDescScroll;
	Boolean descriptionChanged = false;
	Boolean descriptionLocked = false;

	
	
	public ModifyDialog(Task task) {
		_task = task;
		initUI();
	}

	@Override
	public void dispose() {
		super.dispose();

	}

	@SuppressWarnings("unchecked")
	public final void initUI() {

		setLayout(null);
		UIManager.put("TextArea.margin", new Insets(10, 10, 10, 10));

		JLabel lbl1 = new JLabel("Name ");
		JLabel lbl2 = new JLabel("Type ");
		JLabel lbl3 = new JLabel("Dbg-Time   ");
		JLabel lbl4 = new JLabel("Dbg-Result ");

		// Task description
		JLabel lbl5 = new JLabel("Description ");

		if (_task.getTaskDescriptionProvider() != null)
			this.txtName = new JAutoSuggestComboBox(
					_task.getTaskDescriptionProvider().getNamesVector()); // txtName.selectAll();
		else
			this.txtName = new JAutoSuggestComboBox(new Vector<String>()); // txtName.selectAll();

		this.txtName.setEditable(true);
		this.txtName.setEnabled(true);
		this.txtName.setText(_task.text);

		this.cType = new JComboBox(new String[] { Task.TYPE_sequenser,Task.TYPE_selector, Task.TYPE_task, Task.TYPE_parallel, Task.TYPE_switch });
		this.cType.setSelectedItem(_task.type);
		this.txtDbgTime = new JTextField("" + _task.getProperty().testTime);
		this.txtDbgResult = new JComboBox(new String[] { "true","false" });
		this.txtDbgResult.setSelectedItem("" + _task.getProperty().testResult);

		this.txtTaskDescAlgoritm = new JTextArea();
		// txtTaskDescAlgoritm.setAutoscrolls(true);
		this.txtTaskDescScroll = new JScrollPane(this.txtTaskDescAlgoritm);

		this.chkCollapse = new JCheckBox("Collapse");
		this.chkCollapse.setSelected(_task.getProperty().collapsed);

		JButton close = new JButton("Close");
		close.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent event) {
				dispose();
			}
		});

		JButton OK = new JButton("OK");
		OK.addActionListener(new ActionListener() {

			@Override
			public void actionPerformed(ActionEvent event) {
				_task.text = ModifyDialog.this.txtName.getText();
				_task.type = (String) ModifyDialog.this.cType
						.getSelectedItem();
				try {
					_task.getProperty().testTime = Integer
							.parseInt(ModifyDialog.this.txtDbgTime
									.getText());
					_task.getProperty().testResult = Boolean
							.parseBoolean((String) ModifyDialog.this.txtDbgResult
									.getSelectedItem());
					_task.getProperty().collapsed = ModifyDialog.this.chkCollapse
							.isSelected();

					if (_task.type.equalsIgnoreCase(Task.TYPE_task)
							&& _task.getTaskDescriptionProvider() != null) {
						TaskDescription.TaskInfo updateTask = new TaskDescription.TaskInfo();
						updateTask.algorithm = ModifyDialog.this.txtTaskDescAlgoritm
								.getText();
						_task.getTaskDescriptionProvider().put(
								_task.getNameWithoutParameters(), updateTask);
					}
				} catch (Exception e) {
					Log.e(e);
				}
				dispose();
			}
		});

		this.txtName.addActionListener(new ActionListener() {

			@Override
			public void actionPerformed(ActionEvent arg0) {
				if (_task.getTaskDescriptionProvider() == null
						|| ModifyDialog.this.descriptionLocked)
					return;

				String typedText = _task.getNameWithoutParameters(ModifyDialog.this.txtName
						.getText());
				TaskDescription.TaskInfo taskDesc = _task.getTaskDescriptionProvider().get(typedText);

				if (ModifyDialog.this.descriptionChanged
						&& !ModifyDialog.this.txtTaskDescAlgoritm.getText()
								.trim().equals("")) {

					// InputBox
					if (taskDesc != null) {
						int dialogResult = JOptionPane.showConfirmDialog(
								null, "Save description?",
								"Description has changed",
								JOptionPane.YES_NO_OPTION);

						if (dialogResult == JOptionPane.YES_OPTION)
							ModifyDialog.this.descriptionLocked = true;
						else {
							ModifyDialog.this.txtTaskDescAlgoritm
									.setText(taskDesc.algorithm);
						}
					}

				} else {
					if (taskDesc != null)
						ModifyDialog.this.txtTaskDescAlgoritm
								.setText(taskDesc.algorithm);
					else
						ModifyDialog.this.txtTaskDescAlgoritm.setText("");
				}
			}
		});

		this.txtName.addKeyListener(new KeyAdapter() {
			@Override
			public void keyReleased(KeyEvent e) {

			}
		});

		this.txtTaskDescAlgoritm.addKeyListener(new KeyAdapter() {
			@Override
			public void keyReleased(KeyEvent e) {
				ModifyDialog.this.descriptionChanged = true;
			}
		});

		// Handle escape key press
		KeyboardFocusManager.getCurrentKeyboardFocusManager()
				.addKeyEventDispatcher(new KeyEventDispatcher() {
					@Override
					public boolean dispatchKeyEvent(KeyEvent e) {
						if (e.getID() == KeyEvent.KEY_PRESSED) {
							if (e.getKeyCode() == KeyEvent.VK_ESCAPE) {
								dispose();

							}
						}
						return false;
					}
				});
		
		add(lbl1);
		add(this.txtName);
		add(lbl2);
		add(this.cType);

		if (!_task.type.equalsIgnoreCase(Task.TYPE_task)) {
			add(this.chkCollapse);
		} else {
			add(lbl3);
			add(this.txtDbgTime);
			add(lbl4);
			add(this.txtDbgResult);

			if (_task.getTaskDescriptionProvider() != null) {
				TaskDescription.TaskInfo taskDesc = _task.getTaskDescriptionProvider().get(_task.getNameWithoutParameters());

				if (taskDesc != null)
					this.txtTaskDescAlgoritm.setText(taskDesc.algorithm);

				// Task description
				add(lbl5);
				add(this.txtTaskDescScroll);
			}
		}

		add(close);
		add(OK);

		lbl1.setBounds(10, 10, 100, 30);
		this.txtName.setBounds(160, 10, 370, 30);
		lbl2.setBounds(10, 50, 100, 30);
		this.cType.setBounds(160, 50, 370, 30);

		lbl3.setBounds(10, 90, 130, 30);
		this.txtDbgTime.setBounds(160, 90, 370, 30);
		lbl4.setBounds(10, 130, 130, 30);
		this.txtDbgResult.setBounds(160, 130, 370, 30);

		lbl5.setBounds(10, 170, 100, 30);
		// txtTaskDescAlgoritm.setBounds(160, 170, 370, 150);
		this.txtTaskDescScroll.setPreferredSize(new Dimension(370, 150));
		this.txtTaskDescScroll.setBounds(160, 170, 370, 150);
		this.txtTaskDescScroll
				.setHorizontalScrollBarPolicy(ScrollPaneConstants.HORIZONTAL_SCROLLBAR_NEVER);
		this.txtTaskDescAlgoritm.setLineWrap(true);
		// txtTaskDescAlgoritm.setBorder(BorderFactory.createLineBorder(Color.DARK_GRAY));

		this.chkCollapse.setBounds(5, 200, 100, 30);

		close.setBounds(10, 340, 80, 30);
		OK.setBounds(440, 330, 100, 30);

		setModalityType(ModalityType.APPLICATION_MODAL);

		setTitle("Change Task");
		setDefaultCloseOperation(DISPOSE_ON_CLOSE);
		setLocationRelativeTo(null);
		// setSize(300, 230);
		setSize(550, 400);
		
		addWindowListener(new WindowListener() {
			@Override
			public void windowOpened(WindowEvent arg0) {}
			@Override
			public void windowIconified(WindowEvent arg0) {}
			@Override
			public void windowDeiconified(WindowEvent arg0) {}
			@Override
			public void windowDeactivated(WindowEvent arg0) {}
			@Override
			public void windowClosing(WindowEvent arg0) {}
			@Override
			public void windowClosed(WindowEvent arg0) {
				if (_task.getDocument() != null)
					_task.getDocument().repaint();
			}
			@Override
			public void windowActivated(WindowEvent arg0) {}
		});
	}
}
