package document.actions;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import document.BTDesigner;
import document.Document;
import document.Parameters;

public class SaveXMLAction extends AbstractDesignerAction implements
		ActionListener {

	public SaveXMLAction(BTDesigner designer) {
		super(designer);
	}

	@Override
	public void actionPerformed(ActionEvent e) {
		String fileName = null;

		if (e.getActionCommand().equals("file_save_as")) {
			fileName = Dialogs.saveFile("Save XML", "xml", "plan.xml",
					Parameters.path_to_plans);

		} else if (e.getActionCommand().equals("file_save")) {
			Document document = getActiveTab().doc;
			fileName = document.getAbsoluteFilePath();
		}

		if (fileName == null) {
			return;
		}
		savePlan(fileName);

	}

	private void savePlan(String path) {
		// JOptionPane.showMessageDialog(null, "Error: not implemented",
		// "Save Plan",
		// JOptionPane.ERROR_MESSAGE);
		// return;
		getActiveTab().doc.compile(path, true, true);
		// document.setCurrentWorkingFile(path);
	}
}
