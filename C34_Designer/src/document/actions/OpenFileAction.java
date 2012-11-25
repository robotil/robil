package document.actions;

import document.BTDesigner;
import document.Document;
import document.Parameters;

import java.awt.FileDialog;
import java.awt.Frame;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.File;
import java.io.FilenameFilter;

import javax.swing.JFileChooser;
import javax.swing.JOptionPane;
import javax.swing.filechooser.FileNameExtensionFilter;

public class OpenFileAction extends AbstractDesignerAction implements
		ActionListener {

	public OpenFileAction(BTDesigner designer) {
		super(designer);
	}

	public void actionPerformed(ActionEvent a) {

		if (designer.getNumberOfDocuments() == 0) {
			JOptionPane.showMessageDialog(null, "Please open a new window",
					"Open Plan", JOptionPane.INFORMATION_MESSAGE);
			return;
		}

		String fileName = Dialogs.openFile("Open XML", "xml",
				Parameters.path_to_plans);
		if (fileName == null) {
			return;
		}

		Document document = super.getActiveTab().doc;
		
		document.loadPlan(fileName);

		String shortName = document.getShortFilePath();
		designer.setTabName(designer.tabbedPane.getSelectedIndex(), shortName);
	}
}