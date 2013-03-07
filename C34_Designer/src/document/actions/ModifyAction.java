package document.actions;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import windows.designer.BTDesigner;

import document.Document;
import document.ui.Toolbar;

public class ModifyAction extends AbstractDesignerAction implements
		ActionListener {

	public ModifyAction(BTDesigner designer) {
		super(designer);
	}

	@Override
	public void actionPerformed(ActionEvent a) {

		Document document = getActiveTab().doc;

		document.toolSelectionClean();
		document.modifier = new elements.Modifier();

		this.designer.toolbar.setTipText(Toolbar.TIP_modify);
	}
}
