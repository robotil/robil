package document.listeners;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import document.BTDesigner;
import document.Document;



public class RunAction extends AbstractDesignerAction implements ActionListener {
	
	public RunAction(BTDesigner designer) {
		super(designer);
	}
	
	public void actionPerformed(ActionEvent a) {
		getActiveDocument().run();
	}	
}