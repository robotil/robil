package document.listeners;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import document.Document;

public class CompileAction implements ActionListener {
	
	private Document document;
	
	public CompileAction(Document document) {
		this.document = document;
	}
	
	public void actionPerformed(ActionEvent a) {
		document.compile();
	}	
}