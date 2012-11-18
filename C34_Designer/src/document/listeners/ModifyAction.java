package document.listeners;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import document.Document;
import document.Toolbar;

public class ModifyAction implements ActionListener {

	private Document document;
	private Toolbar toolbar;
	
	public ModifyAction(Document document, Toolbar toolbar) {
		this.document = document;
		this.toolbar = toolbar;
	}

	@Override
	public void actionPerformed(ActionEvent a) {
		document.toolSelectionClean();
		document.modifier= new elements.Modifier();
		toolbar.setTipText(Toolbar.TIP_modify);
	}
}
