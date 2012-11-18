package document.listeners;

import document.Document;
import document.Parameters;

import java.awt.FileDialog;
import java.awt.Frame;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.File;
import java.io.FilenameFilter;

import javax.swing.JFileChooser;
import javax.swing.filechooser.FileNameExtensionFilter;

public class OpenFileAction implements ActionListener {

	private Document document;
	
	public OpenFileAction(Document document) {
		this.document = document;
	}
	
	public void actionPerformed(ActionEvent a) {

		String fileName = Dialogs.openFile("Open XML", "xml", Parameters.path_to_plans);
		if (fileName == null) {
			return;
		}
		
		document.loadPlan(fileName);		
	}
}