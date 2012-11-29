package document.actions;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import document.BTDesigner;
import document.Document;



public class TestAction extends AbstractDesignerAction implements ActionListener {
	
	public TestAction(BTDesigner designer) {
		super(designer);
	}
	
	public void actionPerformed(ActionEvent a) {
		getActiveTab().doc.test();
	}	
}