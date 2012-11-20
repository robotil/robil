package document.listeners;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import document.BTDesigner;
import document.Document;
import document.Parameters;
import document.Toolbar;

public class SaveImageAction extends AbstractDesignerAction implements ActionListener {
	
	@Override
	public void actionPerformed(ActionEvent e) {
		String fileName = Dialogs.saveFile("Save Image", "png", "plan.png", Parameters.path_to_images);
		if (fileName == null) {
			return;
		}
		
		Document document = getActiveDocument();
		
		try {	
			Toolbar.getSaveSnapShot( document, fileName);			
		} catch (Exception ex) {
			// TODO Auto-generated catch block
			ex.printStackTrace();
		}
	}

	public SaveImageAction(BTDesigner designer) {
		super(designer);
	}	
}
