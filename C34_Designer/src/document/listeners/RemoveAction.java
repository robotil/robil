package document.listeners;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import document.BTDesigner;
import document.Document;
import document.Toolbar;

public class RemoveAction extends AbstractDesignerAction implements ActionListener {
	
	public RemoveAction(BTDesigner designer) {
		super(designer);
	}
	
	public void actionPerformed(ActionEvent a) {
		Document document = getActiveDocument();
		document.toolSelectionClean();
		document.removeElement = true;
		designer.toolbar.setTipText(Toolbar.TIP_remove);
	}	

}
