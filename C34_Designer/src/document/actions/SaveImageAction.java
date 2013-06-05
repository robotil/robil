package document.actions;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import windows.designer.BTDesigner;

import logger.Log;

import document.Document;
import document.Parameters;
import document.ui.Toolbar;

public class SaveImageAction extends AbstractDesignerAction implements
		ActionListener {

	public SaveImageAction(BTDesigner designer) {
		super(designer);
	}

	@Override
	public void actionPerformed(ActionEvent e) {
		String fileName = Dialogs.saveFile("Save Image", "png", "plan.png",
				Parameters.path_to_images);
		if (fileName == null) {
			return;
		}

		Document document = getActiveTab().document;

		try {
			Toolbar.getSaveSnapShot(document, fileName);
		} catch (Exception ex) {
			// TODO Auto-generated catch block
			Log.e(ex);
		}
	}
}
