package document.actions;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import windows.designer.BTDesigner;

import document.Document;
import elements.GElement;

public class ToolAction extends AbstractDesignerAction implements
		ActionListener {

	private GElement.Creator c = null;

	public ToolAction(BTDesigner designer, GElement.Creator c) {
		super(designer);
		this.c = c;
	}

	@Override
	public void actionPerformed(ActionEvent a) {
		Document document = getActiveTab().document;
		document.toolSelectionClean();
		document.creator = this.c;
		this.designer.toolbar.setTipText(this.c.toolTip());
	}
}
