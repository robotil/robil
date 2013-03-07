package document.actions;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import windows.designer.BTDesigner;

import document.Document;
import document.ui.Toolbar;

public class RemoveAction extends AbstractDesignerAction implements
		ActionListener {

	public RemoveAction(BTDesigner designer) {
		super(designer);
	}

	@Override
	public void actionPerformed(ActionEvent a) {
		Document document = getActiveTab().doc;
		document.toolSelectionClean();
		document.removeElement = true;
		this.designer.toolbar.setTipText(Toolbar.TIP_remove);
	}

}
