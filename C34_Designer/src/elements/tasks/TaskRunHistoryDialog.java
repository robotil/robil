package elements.tasks;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.Insets;
import java.awt.Shape;
import java.util.UUID;

import javax.swing.JDialog;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTextArea;
import javax.swing.JTextField;
import javax.swing.UIManager;
import javax.swing.text.DefaultHighlighter;
import javax.swing.text.Highlighter.HighlightPainter;
import javax.swing.text.JTextComponent;
import javax.swing.text.StyledDocument;

public class TaskRunHistoryDialog extends JDialog {
	private static final long serialVersionUID = -4309214496648164067L;
	
	private final Task _task;
	
	public TaskRunHistoryDialog(JFrame parent, Task task) {
		this._task = task;
		
		build();
		setModalityType(ModalityType.APPLICATION_MODAL);
		
		if (parent != null)
			setLocationRelativeTo(parent);
		else
			setLocation(100, 100);
		
		
		setTitle("Task run history");
		setDefaultCloseOperation(DISPOSE_ON_CLOSE);
		setSize(600, 400);
		setVisible(true);
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
		JTextArea txtOutput = new JTextArea();
		
		txtOutput.setEditable(false);
		for (TaskResult result : _task.getRunResults())
			txtOutput.append(result + "\n");
		
		// txtOutput.getHighlighter().addHighlight(0, 10, new DefaultHighlighter.DefaultHighlightPainter(Color.RED)));
		
		txtOutput.setCaretPosition(txtOutput.getDocument().getLength());

		panel.add(new JScrollPane(txtOutput), c);
		
		add(panel);
	}
	
	public static void main(String[] args) {
		Task task = new Task();
		task.id = new UUID(123123123, 12312312);
		
		new TaskRunHistoryDialog(null, task);
	}
}
