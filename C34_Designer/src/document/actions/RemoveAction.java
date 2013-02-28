package document.actions;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import document.BTDesigner;
import document.Document;
import document.Toolbar;

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
