package document.actions;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import logger.Log;

import document.BTDesigner;
import document.Document;
import document.Parameters;
import document.Toolbar;

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

		Document document = getActiveTab().doc;

		try {
			Toolbar.getSaveSnapShot(document, fileName);
		} catch (Exception ex) {
			// TODO Auto-generated catch block
			Log.e(ex);
		}
	}
}
