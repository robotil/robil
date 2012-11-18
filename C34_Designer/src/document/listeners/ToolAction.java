package document.listeners;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import document.Document;
import document.Toolbar;

import elements.GElement;


public class ToolAction implements ActionListener {
	
	private Document document;
	private GElement.Creator c = null;
	private Toolbar toolbar;
	
	public ToolAction(Document document, Toolbar toolbar, GElement.Creator c) {
		this.c = c;
		this.document = document;
		this.toolbar = toolbar;
	}


	public void actionPerformed(ActionEvent a) {
		document.toolSelectionClean();
		document.creator = c;
		toolbar.setTipText(c.toolTip());
	}
}
