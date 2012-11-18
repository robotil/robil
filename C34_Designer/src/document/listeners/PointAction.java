package document.listeners;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import document.Document;
import document.Toolbar;

public class PointAction implements ActionListener {

	private Document document;
	private Toolbar toolbar;
	
	public PointAction(Document document, Toolbar toolbar) {
		this.document = document;
		this.toolbar = toolbar;
	}

	public void actionPerformed(ActionEvent a) {
		document.toolSelectionClean();
		toolbar.setTipText(Toolbar.TIP_move);
	}

}
