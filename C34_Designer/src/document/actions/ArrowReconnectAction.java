package document.actions;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import windows.designer.BTDesigner;

import document.Document;
import document.ui.Toolbar;

public class ArrowReconnectAction extends AbstractDesignerAction implements
		ActionListener {

	public ArrowReconnectAction(BTDesigner designer) {
		super(designer);
	}

	@Override
	public void actionPerformed(ActionEvent a) {
		Document document = getActiveTab().document;
		document.toolSelectionClean();
		document.reconectArrow = true;
		this.designer.toolbar.setTipText(Toolbar.TIP_reconnect);
	}

}
