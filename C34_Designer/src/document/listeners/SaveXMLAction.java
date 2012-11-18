package document.listeners;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.JDialog;
import javax.swing.JOptionPane;

import document.Document;
import document.Parameters;

public class SaveXMLAction implements ActionListener {
	
	private Document document;
	
	private void savePlan(String path) {
		JOptionPane.showMessageDialog(null, "Error: not implemented", "Save Plan",
				JOptionPane.ERROR_MESSAGE);
		return;
//		document.setCurrentWorkingFile(path);
	}
	
	@Override
	public void actionPerformed(ActionEvent e) {
		String fileName = null;
		switch (e.getActionCommand()) {
		case "file_save_as":
			
			fileName = Dialogs.saveFile("Save XML", "xml", "plan.xml", Parameters.path_to_plans);
			
			break;
		
		case "file_save":
			fileName = document.getAbsoluteFilePath();
			break;
		}
		
		if (fileName == null) {
			return;
		}			
		savePlan(fileName);
		
	}

	public void SaveFileActionListener(Document document){
		this.document = document;
	}	
}
