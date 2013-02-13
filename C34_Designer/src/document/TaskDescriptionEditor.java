package document;

import java.awt.Dimension;
import java.awt.Font;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.Insets;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.DefaultListModel;
import javax.swing.ImageIcon;
import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JList;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTabbedPane;
import javax.swing.JTextArea;
import javax.swing.JTextField;
import javax.swing.ListSelectionModel;
import javax.swing.SwingUtilities;
import javax.swing.UIManager;
import javax.swing.UnsupportedLookAndFeelException;
import javax.swing.event.ListSelectionEvent;
import javax.swing.event.ListSelectionListener;

import document.description.TaskDescription;

public class TaskDescriptionEditor extends JFrame {

	private static final long serialVersionUID = 844706702028498340L;

	public static void main(String[] args) throws ClassNotFoundException,
			InstantiationException, IllegalAccessException,
			UnsupportedLookAndFeelException {

		SwingUtilities.invokeLater(new Runnable() {

			@Override
			public void run() {
				TaskDescription tasks = new TaskDescription();

				TaskDescriptionEditor editor = new TaskDescriptionEditor(tasks);
				editor.setVisible(true);
			}
		});
	}

	private JTextField _txtEditTaskName = new JTextField();

	private JTextArea _txtEditTaskDescription = new JTextArea();
	private JTextField _txtCreateTaskName = new JTextField();

	private JTextArea _txtCreateTaskDescription = new JTextArea();
	//VERSION_PROBLME private JList<String> _lstTasks = new JList<String>();
	private JList _lstTasks = new JList();

	private TaskDescription _tasks;

	public TaskDescriptionEditor(TaskDescription tasks) {

		for (javax.swing.UIManager.LookAndFeelInfo info : javax.swing.UIManager
				.getInstalledLookAndFeels()) {
			if ("GTK+".equals(info.getName())) {
				try {
					javax.swing.UIManager.setLookAndFeel(info.getClassName());
				} catch (Exception e) {
					e.printStackTrace();
				}
				break;
			}
		}

		UIManager.put("TextArea.margin", new Insets(10, 10, 10, 10));

		this._tasks = tasks;

		setLayout(new GridBagLayout());
		GridBagConstraints c = new GridBagConstraints();

		c.insets = new Insets(10, 10, 10, 10);

		c.fill = GridBagConstraints.HORIZONTAL;
		c.anchor = GridBagConstraints.WEST;
		c.gridx = 0;
		c.gridy = 0;
		c.weightx = 0.0;
		c.weighty = 0.0;
		add(new JLabel("Filename: " + tasks.getFilename()), c);

		c.fill = GridBagConstraints.BOTH;
		c.gridx = 0;
		c.gridy = 1;
		c.weightx = 1.0;
		c.weighty = 1.0;
		add(createTabs(), c);

		c.fill = GridBagConstraints.NONE;
		c.anchor = GridBagConstraints.EAST;
		c.gridx = 0;
		c.gridy = 2;
		c.weightx = 0.0;
		c.weighty = 0.0;

		add(new JButton("   Close   ") {
			{
				this.setPreferredSize(new Dimension(150, 42));
				Font font = new Font(getFont().getName(), Font.BOLD, 16);
				this.setFont(font);
			}
			private static final long serialVersionUID = -5658657254282091109L;
			{
				addActionListener(new ActionListener() {
					@Override
					public void actionPerformed(ActionEvent e) {
						dispose();
					}
				});
			}
		}, c);

		Insets insets = new Insets(5, 5, 5, 5);
		this._txtCreateTaskDescription.setMargin(insets);
		this._txtCreateTaskName.setMargin(insets);
		this._txtEditTaskDescription.setMargin(insets);
		this._txtEditTaskName.setMargin(insets);

		refreshTasks();

		setTitle("Task description editor");
		setSize(800, 550);

		setLocationRelativeTo(null);
	}

	private JPanel createEditPanel() {
		JPanel panel = new JPanel();
		panel.setLayout(new GridBagLayout());
		GridBagConstraints c = new GridBagConstraints();

		c.insets = new Insets(0, 10, 10, 10);
		c.fill = GridBagConstraints.BOTH;
		c.gridx = 0;
		c.gridy = 0;
		c.weightx = 0.5;
		c.weighty = 0.0;
		panel.add(new JLabel("Task name:"), c);

		c.gridy = 1;

		this._txtEditTaskName.setEditable(false);
		panel.add(this._txtEditTaskName, c);

		c.gridy = 2;
		panel.add(new JLabel("Task description"), c);

		c.gridy = 3;
		c.weighty = 1.0;
		this._txtEditTaskDescription = new JTextArea();
		this._txtEditTaskDescription.setEditable(true);
		this._txtEditTaskDescription.setLineWrap(true);
		panel.add(new JScrollPane(this._txtEditTaskDescription), c);

		c.gridy = 4;
		c.weighty = 0.0;
		c.fill = GridBagConstraints.NONE;
		c.anchor = GridBagConstraints.EAST;
		panel.add(new JButton("   Save description   ") {
			private static final long serialVersionUID = 8563475305831999942L;
			{
				{
					ImageIcon icon = new ImageIcon(getClass().getClassLoader()
							.getResource("icons/save.png"));
					this.setIcon(icon);
				}
				addActionListener(new ActionListener() {
					@Override
					public void actionPerformed(ActionEvent e) {
						saveTask(
								TaskDescriptionEditor.this._txtEditTaskName
										.getText(),
								TaskDescriptionEditor.this._txtEditTaskDescription
										.getText());
					}
				});
			}
		}, c);

		return panel;
	}

	private JTabbedPane createTabs() {
		JTabbedPane tabs = new JTabbedPane();
		tabs.addTab("Edit task descriptions", new ImageIcon(getClass()
				.getClassLoader().getResource("icons/modify.png")),
				createViewEditPanel());
		tabs.addTab("Create new task description", new ImageIcon(getClass()
				.getClassLoader().getResource("icons/new_tab.png")),
				createTaskCreatePanel());

		return tabs;
	}

	private void createTask(String taskName, String taskDescription) {
		if (taskName.trim().equals("")) {
			JOptionPane.showMessageDialog(this,
					"Task name is empty, please type task name");
			return;
		}

		if (this._tasks.get(taskName) != null) {
			JOptionPane
					.showMessageDialog(this, "Specified task already exists");
			return;
		}

		this._tasks.addTaskDescription(taskName, taskDescription);
	}

	private JPanel createTaskCreatePanel() {
		JPanel panel = new JPanel();
		panel.setLayout(new GridBagLayout());
		GridBagConstraints c = new GridBagConstraints();

		c.insets = new Insets(10, 20, 0, 20);
		c.fill = GridBagConstraints.BOTH;
		c.gridx = 0;
		c.gridy = 0;
		c.weightx = 0.5;
		c.weighty = 0.0;
		panel.add(new JLabel("Task name"), c);

		c.gridy = 1;
		panel.add(this._txtCreateTaskName, c);

		c.gridy = 2;
		panel.add(new JLabel("Task description"), c);

		c.gridy = 3;
		c.weighty = 1.0;
		this._txtCreateTaskDescription = new JTextArea();
		this._txtCreateTaskDescription.setEditable(true);
		this._txtCreateTaskDescription.setLineWrap(true);
		panel.add(new JScrollPane(this._txtCreateTaskDescription), c);

		c.gridy = 4;
		c.weighty = 0.0;
		c.fill = GridBagConstraints.NONE;
		c.anchor = GridBagConstraints.EAST;
		c.insets = new Insets(10, 20, 20, 20);
		panel.add(new JButton("   Save description   ") {
			{
				ImageIcon icon = new ImageIcon(getClass().getClassLoader()
						.getResource("icons/save.png"));
				this.setIcon(icon);
			}
			private static final long serialVersionUID = 8563475305831999942L;
			{
				{
					ImageIcon icon = new ImageIcon(getClass().getClassLoader()
							.getResource("icons/save.png"));
					this.setIcon(icon);
				}
				addActionListener(new ActionListener() {
					@Override
					public void actionPerformed(ActionEvent e) {
						createTask(
								TaskDescriptionEditor.this._txtCreateTaskName
										.getText(),
								TaskDescriptionEditor.this._txtCreateTaskDescription
										.getText());
					}
				});
			}
		}, c);

		return panel;
	}

	private JPanel createTasksList() {
		JPanel panel = new JPanel();
		panel.setLayout(new GridBagLayout());
		GridBagConstraints c = new GridBagConstraints();

		//VERSION_PROBLME this._lstTasks = new JList<String>();
		this._lstTasks = new JList();
		this._lstTasks.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);
		this._lstTasks.addListSelectionListener(new ListSelectionListener() {

			@Override
			public void valueChanged(ListSelectionEvent e) {
				//VERSION_PROBLME selectTask(TaskDescriptionEditor.this._lstTasks	.getSelectedValue());
				selectTask(TaskDescriptionEditor.this._lstTasks	.getSelectedValue().toString());
			}
		});

		JScrollPane listScroll = new JScrollPane(this._lstTasks);

		c.insets = new Insets(0, 10, 10, 10);

		c.anchor = GridBagConstraints.SOUTHWEST;
		c.fill = GridBagConstraints.NONE;
		c.gridy = 0;
		c.weightx = 0.0;
		c.weighty = 0.0;
		panel.add(new JLabel("Available descriptions:"), c);

		c.fill = GridBagConstraints.BOTH;
		c.gridx = 0;
		c.gridy = 1;
		c.weightx = 0.0;
		c.weighty = 1.0;

		panel.add(listScroll, c);

		c.gridx = 0;
		c.gridy = 2;
		c.weighty = 0.0;
		c.weightx = 1.0;

		c.fill = GridBagConstraints.NONE;
		c.anchor = GridBagConstraints.LAST_LINE_END;
		c.insets = new Insets(0, 10, 10, 10);

		panel.add(new JButton("Delete selected task") {
			private static final long serialVersionUID = -2603573058085961695L;
			{
				{
					ImageIcon icon = new ImageIcon(getClass().getClassLoader()
							.getResource("icons/remove.png"));
					this.setIcon(icon);
				}
				addActionListener(new ActionListener() {
					@Override
					public void actionPerformed(ActionEvent e) {
						//VERSION_PROBLME deleteTask(TaskDescriptionEditor.this._lstTasks	.getSelectedValue());
						deleteTask(TaskDescriptionEditor.this._lstTasks	.getSelectedValue().toString());
					}
				});
			}
		}, c);

		panel.setMinimumSize(new Dimension(250, 400));
		return panel;
	}

	private JPanel createViewEditPanel() {
		JPanel panel = new JPanel();

		panel.setLayout(new GridBagLayout());
		GridBagConstraints c = new GridBagConstraints();

		c.insets = new Insets(10, 10, 10, 10);
		c.fill = GridBagConstraints.VERTICAL;
		c.gridx = 0;
		c.gridy = 0;
		c.weightx = 0.0;
		c.weighty = 1.0;

		panel.add(createTasksList(), c);

		c.fill = GridBagConstraints.BOTH;
		c.gridx = 1;
		c.gridy = 0;
		c.weightx = 1.0;
		c.weighty = 1.0;
		c.anchor = GridBagConstraints.SOUTHWEST;
		panel.add(createEditPanel(), c);

		return panel;
	}

	private void deleteTask(String taskName) {
		if (taskName.equals("")) {
			JOptionPane.showMessageDialog(this,
					"Select task from the list above");
			return;
		}

		if (JOptionPane
				.showConfirmDialog(
						this,
						String.format(
								"You are about to remove description for task '%s', are you sure?",
								taskName), "Task description removal",
						JOptionPane.YES_NO_OPTION) == JOptionPane.YES_OPTION) {
			this._tasks.remove(taskName);
			refreshTasks();
		}
	}

	private void refreshTasks() {
		//VERSION_PROBLME DefaultListModel<String> listModel = new DefaultListModel<String>();
		DefaultListModel listModel = new DefaultListModel();

		for (String taskName : this._tasks.getNames())
			if (!taskName.trim().equals(""))
				listModel.addElement(taskName);

		this._lstTasks.setModel(listModel);

		this._txtEditTaskDescription.setText("");
		this._txtEditTaskName.setText("");
	}

	private void saveTask(String taskName, String taskDescription) {
		this._tasks.update(taskName, taskDescription);
		// refreshTasks();
	}

	private void selectTask(String taskName) {
		this._txtEditTaskName.setText(taskName);
		this._txtEditTaskDescription
				.setText(this._tasks.get(taskName).algorithm);
	}
}
