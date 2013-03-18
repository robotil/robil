package document.actions;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import windows.designer.BTDesigner;

public class OpenLookupTableEditorAction implements ActionListener {

	private BTDesigner _designer; 
	
	public OpenLookupTableEditorAction(BTDesigner designer) {
		this._designer = designer;
	}
	
	@Override
	public void actionPerformed(ActionEvent arg) {
		this._designer.showLookupTableEditor();
	}

}
