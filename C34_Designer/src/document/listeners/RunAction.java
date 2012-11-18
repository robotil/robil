package document.listeners;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import document.Document;



public class RunAction implements ActionListener {
	
	private Document document;
	
	public RunAction(Document document) {
		this.document = document;
	}
	
	public void actionPerformed(ActionEvent a) {
		document.run();
	}	
}