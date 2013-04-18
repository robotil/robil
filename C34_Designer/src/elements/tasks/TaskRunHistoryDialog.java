package elements.tasks;

import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.Insets;

import javax.swing.JDialog;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTextArea;
import javax.swing.JTextField;
import javax.swing.UIManager;

import document.PlanExecution;

import elements.tasks.TaskResultCollection.EventListenerArgs;

public class TaskRunHistoryDialog extends JDialog {
	private static final long serialVersionUID = -4309214496648164067L;
	
	private final Task _task;
	private JTextArea _txtOutput;
	private PlanExecution _lastPlan;
	
	public TaskRunHistoryDialog(JFrame parent, Task task) {
		this._task = task;
		
		build();
		// setModalityType(ModalityType.APPLICATION_MODAL);
		
		if (parent != null)
			setLocationRelativeTo(parent);
		else
			setLocation(100, 100);
		
		setEventListener();
		
		setTitle("Task run history");
		setDefaultCloseOperation(DISPOSE_ON_CLOSE);
		setSize(600, 400);
		setVisible(true);
	}

	private void setEventListener() {
		_task.getRunResults().addEventListener(new TaskResultCollection.EventListener() {
			@Override
			public void onAdd(EventListenerArgs e) {
				if (e.taskResult.getPlanExecution() != _lastPlan) {
					_txtOutput.append("------------\n");
					_lastPlan = e.taskResult.getPlanExecution();
				}
				
				_txtOutput.append(e.taskResult.toString() + "\n");
				_txtOutput.setCaretPosition(_txtOutput.getDocument().getLength());
			}
		});
	}

	private void build() {
		UIManager.put("TextArea.margin", new Insets(5, 5, 5, 5));
		JPanel panel = new JPanel(); 
		panel.setLayout(new GridBagLayout());
		GridBagConstraints c = new GridBagConstraints();

		c.insets = new Insets(10, 10, 5, 10);
		c.fill = GridBagConstraints.HORIZONTAL;
		c.gridx = 0;
		c.gridy = 0;
		c.weightx = 1;
		panel.add(new JLabel("Task id:"), c);
		
		c.insets = new Insets(0, 10, 10, 10);
		c.fill = GridBagConstraints.HORIZONTAL;
		c.gridx = 0;
		c.gridy++;
		c.weightx = 1;
		JTextField txtTaskId = new JTextField(_task.id.toString());
		txtTaskId.setMargin(new Insets(5, 5, 5, 5));
		txtTaskId.setEditable(false);
		panel.add(txtTaskId, c);
		
		c.insets = new Insets(10, 10, 5, 10);
		c.fill = GridBagConstraints.HORIZONTAL;
		c.gridx = 0;
		c.gridy++;
		c.weightx = 1;
		panel.add(new JLabel("Last results:"), c);
		
		c.insets = new Insets(0, 10, 10, 10);
		c.fill = GridBagConstraints.BOTH;
		c.gridx = 0;
		c.gridy++;
		c.weightx = 1;
		c.weighty = 1;
		
		// TextArea
		_txtOutput = new JTextArea();
		
		_txtOutput.setEditable(false);
		
		PlanExecution prevPlan = null;
		for (TaskResult result : _task.getRunResults()) {
			if (prevPlan != result.getPlanExecution()) {
				_txtOutput.append("------------\n");
				prevPlan = result.getPlanExecution();
			}
				
			_txtOutput.append(result + "\n");
			_lastPlan = result.getPlanExecution();
		}
		
		_txtOutput.setCaretPosition(_txtOutput.getDocument().getLength());

		panel.add(new JScrollPane(_txtOutput), c);
		
		add(panel);
	}
	
}
