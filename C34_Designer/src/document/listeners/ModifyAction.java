package document.listeners;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import document.BTDesigner;
import document.Document;
import document.Toolbar;

public class ModifyAction extends AbstractDesignerAction implements ActionListener {
	
	public ModifyAction(BTDesigner designer) {
		super(designer);
	}

	@Override
	public void actionPerformed(ActionEvent a) {
		
		Document document = getActiveDocument();
		
		document.toolSelectionClean();
		document.modifier= new elements.Modifier();
				
		designer.toolbar.setTipText(Toolbar.TIP_modify);
	}
}
