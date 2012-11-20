package document.listeners;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import document.BTDesigner;
import document.Document;

import elements.GElement;

public class ToolAction extends AbstractDesignerAction implements
		ActionListener {

	private GElement.Creator c = null;

	public ToolAction(BTDesigner designer, GElement.Creator c) {
		super(designer);
		this.c = c;
	}

	public void actionPerformed(ActionEvent a) {
		Document document = getActiveDocument();
		document.toolSelectionClean();
		document.creator = c;
		designer.toolbar.setTipText(c.toolTip());
	}
}
