package document.actions;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import document.BTDesigner;
import document.Document;
import document.Toolbar;

public class CopyTreeAction extends AbstractDesignerAction implements ActionListener {
	
	public CopyTreeAction(BTDesigner designer) {
		super(designer);
	}
	
	public void actionPerformed(ActionEvent a) {
		Document document = getActiveTab().doc;
		document.toolSelectionClean();
		document.copyElement = true;
		designer.toolbar.setTipText(Toolbar.TIP_copy);
	}	

}
