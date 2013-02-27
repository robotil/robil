package document.actions;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import document.BTDesigner;

public class OpenEditorAction implements ActionListener {

	private BTDesigner _designer; 
	
	public OpenEditorAction(BTDesigner designer) {
		this._designer = designer;
	}
	
	@Override
	public void actionPerformed(ActionEvent arg) {
		this._designer.getActiveTab().doc.showEditor();
	}

}
