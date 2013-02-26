package document;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.Insets;
import java.io.BufferedWriter;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Scanner;

import javax.swing.JButton;
import javax.swing.JDialog;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTextField;
import javax.swing.JTextPane;
import javax.swing.UIManager;

import logger.Log;

public class PlanEditor extends JDialog {
	
	private static final long serialVersionUID = -4309214496648164067L;
	
	private final Document _document;
	private JTextPane _txtOutput;
	
	public PlanEditor(JFrame parent, Document document) {
		super(parent, ModalityType.APPLICATION_MODAL);
		this._document = document;
		
		build();
		openXml(document.getAbsoluteFilePath());
		
		if (parent != null)
			setLocationRelativeTo(parent);
		else
			setLocation(100, 100);

		
		setTitle("Plan XML editor");
		setDefaultCloseOperation(DISPOSE_ON_CLOSE);
		setSize(1000, 700);
		setVisible(true);
		

	}
	
	public void openXml(String path) {
		try {
			Scanner scanner = new Scanner(new FileReader(path));
			StringBuilder sb = new StringBuilder();
			
			while (scanner.hasNextLine())
				sb.append(scanner.nextLine() + "\n");
			
			_txtOutput.setText(sb.toString());
			
			scanner.close();
				
		} catch (FileNotFoundException e) {
			Log.e("Couldn't open xml file " + path);
		}
		
	}
	
	public void writeXml(String path, String xml) {
		try {
	        BufferedWriter out = new BufferedWriter(new FileWriter(path));
	        out.write(xml);
	        out.close();
	    } catch (IOException e) {
	    	Log.e("Couldn't write to xml file " + path);
	    }
	}
	
	public void reloadPlan() {
		_document.loadPlan(_document.getAbsoluteFilePath());
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
		panel.add(new JLabel("Plan filename:"), c);
		
		c.insets = new Insets(0, 10, 10, 10);
		c.fill = GridBagConstraints.HORIZONTAL;
		c.gridx = 0;
		c.gridy++;
		c.weightx = 1;
		JTextField txtTaskId = new JTextField(_document.getShortFilePath().toString());
		txtTaskId.setMargin(new Insets(5, 5, 5, 5));
		txtTaskId.setEditable(false);
		panel.add(txtTaskId, c);
		
		c.insets = new Insets(10, 10, 5, 10);
		c.fill = GridBagConstraints.HORIZONTAL;
		c.gridx = 0;
		c.gridy++;
		c.weightx = 1;
		panel.add(new JLabel("XML:"), c);
		
		c.insets = new Insets(0, 10, 10, 10);
		c.fill = GridBagConstraints.BOTH;
		c.gridx = 0;
		c.gridy++;
		c.weightx = 1;
		c.weighty = 1;
		
		// TextArea
		_txtOutput = new JTextPane();
//		_txtOutput.setEditorKitForContentType("text/xml", new XmlEditorKit());
//		_txtOutput.setContentType("text/xml");

		panel.add(new JScrollPane(_txtOutput), c);
		// panel.add(_txtOutput, c);
		
		c.insets = new Insets(0, 10, 10, 10);
		c.fill = GridBagConstraints.NONE;
		c.gridx = 0;
		c.gridy++;
		c.weightx = 0;
		c.weighty = 0;
		c.anchor = GridBagConstraints.EAST;
		
		JButton btnSave = new JButton("Save");
		btnSave.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent arg0) {
				writeXml(_document.getAbsoluteFilePath(), _txtOutput.getText());
				reloadPlan();
				dispose();
			}
		});
		
		panel.add(btnSave, c);
		
		add(panel);
	}
}
