package document.listeners;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import document.Document;
import document.Toolbar;

public class RemoveAction implements ActionListener {

	private Document document;
	private Toolbar toolbar;
	
	public RemoveAction(Document document, Toolbar toolbar) {
		this.document = document;
		this.toolbar = toolbar;
	}
	
	public void actionPerformed(ActionEvent a) {
		document.toolSelectionClean();
		document.removeElement = true;
		toolbar.setTipText(Toolbar.TIP_remove);
	}	

}
